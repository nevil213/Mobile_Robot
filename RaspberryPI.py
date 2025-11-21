import os
import time
import heapq
import cv2
import numpy as np
from picamera2 import Picamera2
from PIL import Image, ImageDraw
from pupil_apriltags import Detector
from collections import deque
import logging
import socket

LOG_FILENAME='dstar_debug.log'
logging.basicConfig(
    filename=LOG_FILENAME,
    filemode='a',
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)

logger = logging.getLogger("DSTAR_LOGGER")

CELL_SIZE_PIX = (50, 50)
STD_THRESHOLD = 15
METERS_PER_PIXEL = 0.001747
TICK_METERS = 0.05
PHOTO_INTERVAL = 1.0
APRILTAG_FAMILY = "tag36h11"
APRILTAG_IDS_ROBOT = [0]

ESP32_IP = "esp32's ip"
UDP_PORT = 4210
BUFFER_SIZE = 1024
TIMEOUT = 2.0

TAG_SIZE_M = 0.05
TAG_FORWARD_OFFSET_DEG = 90

CAM_FX = 920.0
CAM_FY = 920.0
CAM_CX = 640.0
CAM_CY = 360.0
CAMERA_PARAMS = (CAM_FX, CAM_FY, CAM_CX, CAM_CY)

SAVE_PHOTO_DIR = "dstar_photos"
VIS_SAVE_FILE = "dstar_live_overlay.jpg"
os.makedirs(SAVE_PHOTO_DIR, exist_ok=True)

INF = float('inf')

class DStarLite:
    def __init__(self, grid, start, goal):
        self.grid = grid.copy()
        self.rows, self.cols = grid.shape
        self.start = start
        self.goal = goal
        self.g = { (r,c): INF for r in range(self.rows) for c in range(self.cols) }
        self.rhs = { (r,c): INF for r in range(self.rows) for c in range(self.cols) }
        self.U = []
        self.km = 0
        self.rhs[self.goal] = 0
        heapq.heappush(self.U, (self.calculate_key(self.goal), self.goal))

    def in_bounds(self, s): return 0 <= s[0] < self.rows and 0 <= s[1] < self.cols
    def is_free(self, s): return self.in_bounds(s) and self.grid[s] == 0

    def neighbors(self, s):
        r, c = s
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            n = (r+dr, c+dc)
            if self.in_bounds(n):
                yield n

    def heuristic(self, a, b): return abs(a[0]-b[0]) + abs(a[1]-b[1])

    def calculate_key(self, s):
        g_rhs = min(self.g.get(s, INF), self.rhs.get(s, INF))
        return (g_rhs + self.heuristic(self.start, s) + self.km, g_rhs)

    def insert(self, s):
        heapq.heappush(self.U, (self.calculate_key(s), s))

    def remove_from_queue(self, s):
        self.U = [it for it in self.U if it[1] != s]
        heapq.heapify(self.U)

    def update_vertex(self, s):
        if s != self.goal:
            self.rhs[s] = min((self.g.get(n, INF) + 1 for n in self.neighbors(s) if self.is_free(n)), default=INF)
        self.remove_from_queue(s)
        if self.g.get(s, INF) != self.rhs[s]:
            self.insert(s)

    def compute_shortest_path(self, max_iters=200000):
        for _ in range(max_iters):
            if not self.U:
                break
            k_old, u = heapq.heappop(self.U)
            k_new = self.calculate_key(u)
            if k_old < k_new:
                heapq.heappush(self.U, (k_new, u))
            elif self.g.get(u, INF) > self.rhs.get(u, INF):
                self.g[u] = self.rhs[u]
                for s in self.neighbors(u):
                    self.update_vertex(s)
            else:
                self.g[u] = INF
                for s in [u] + list(self.neighbors(u)):
                    self.update_vertex(s)

    def update_map(self, changed_cells):
        self.km += self.heuristic(self.start, self.goal)
        for cell in changed_cells:
            for n in [cell] + list(self.neighbors(cell)):
                if self.in_bounds(n):
                    self.update_vertex(n)
        self.compute_shortest_path()

    def get_shortest_path(self):
        if self.g.get(self.start, INF) == INF:
            return []
        path = [self.start]
        cur = self.start
        while cur != self.goal:
            nbrs = [n for n in self.neighbors(cur) if self.is_free(n)]
            if not nbrs:
                return []
            cur = min(nbrs, key=lambda n: (self.g.get(n, INF), self.heuristic(n, self.goal)))
            path.append(cur)
            if len(path) > (self.rows * self.cols):
                return []
        return path

def frame_to_grid(frame, cell_size=CELL_SIZE_PIX, std_threshold=STD_THRESHOLD, ignore_robot_cells=None):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    cw, ch = cell_size
    rows = (h + ch - 1) // ch
    cols = (w + cw - 1) // cw
    grid = np.zeros((rows, cols), dtype=np.uint8)
    for r in range(rows):
        for c in range(cols):
            y1, y2 = r*ch, min((r+1)*ch, h)
            x1, x2 = c*cw, min((c+1)*cw, w)
            patch = gray[y1:y2, x1:x2]
            if np.std(patch) > std_threshold or np.mean(patch) < 10:
                grid[r, c] = 1
    if ignore_robot_cells:
        for rr, cc in ignore_robot_cells:
            if 0 <= rr < rows and 0 <= cc < cols:
                grid[rr, cc] = 0
                for dr, dc in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                    nr, nc = rr + dr, cc + dc
                    if 0 <= nr < rows and 0 <= nc < cols:
                        grid[nr, nc] = 0
    return grid, rows, cols

def grid_difference(old_grid, new_grid):
    if old_grid is None or old_grid.shape != new_grid.shape:
        return [(r,c) for r in range(new_grid.shape[0]) for c in range(new_grid.shape[1])]
    return [(r,c) for r in range(old_grid.shape[0]) for c in range(old_grid.shape[1]) if old_grid[r,c] != new_grid[r,c]]

detector = Detector(families=APRILTAG_FAMILY)

def detect_robot_with_pose(gray):
    detections = detector.detect(
        gray,
        estimate_tag_pose=True,
        camera_params=CAMERA_PARAMS,
        tag_size=TAG_SIZE_M
    )
    for det in detections:
        if det.tag_id not in APRILTAG_IDS_ROBOT:
            continue
        try:
            R = det.pose_R
            yaw_rad = np.arctan2(R[0,1], R[1,1])
            yaw_deg = np.degrees(yaw_rad)
            robot_yaw_deg = (yaw_deg + TAG_FORWARD_OFFSET_DEG) % 360
            yaw_clockwise = robot_yaw_deg % 360
            facing = int((yaw_clockwise + 45) // 90) % 4
            cx, cy = int(det.center[0]), int(det.center[1])
            cell = (cy // CELL_SIZE_PIX[1], cx // CELL_SIZE_PIX[0])
            logger.info(f"[TAG] Cell: {cell} | Yaw: {robot_yaw_deg:.1f}ÃÂÃÂ° | Facing: {['N','E','S','W'][facing]} | center: ({cx},{cy})")
            return {
                'cell': cell,
                'center_px': (cx, cy),
                'yaw_deg': robot_yaw_deg,
                'facing': facing
            }
        except Exception as e:
            logger.info(f"[POSE ERROR] {e}")
    return None

def overlay_visual(frame, grid, cell_size, path=None, robot=None, changed=None):
    img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).convert("RGBA")
    draw = ImageDraw.Draw(img, 'RGBA')
    h, w = frame.shape[:2]
    cw, ch = cell_size
    rows, cols = grid.shape

    for r in range(rows + 1):
        y = min(r * ch, h)
        draw.line([(0, y), (w, y)], fill=(0, 255, 0, 120), width=1)
    for c in range(cols + 1):
        x = min(c * cw, w)
        draw.line([(x, 0), (x, h)], fill=(0, 255, 0, 120), width=1)

    for r in range(rows):
        for c in range(cols):
            if grid[r, c]:
                draw.rectangle([c*cw, r*ch, min(c*cw+cw, w), min(r*ch+ch, h)], fill=(255, 0, 0, 100))

    if path:
        for r, c in path:
            draw.rectangle([c*cw + cw//6, r*ch + ch//6, c*cw + 5*cw//6, r*ch + 5*ch//6], fill=(0, 200, 0, 150))

    if changed:
        for r, c in changed:
            draw.rectangle([c*cw, r*ch, min(c*cw+cw, w), min(r*ch+ch, h)], outline=(255, 255, 0, 200), width=3)

    vis = cv2.cvtColor(np.array(img.convert("RGB")), cv2.COLOR_RGB2BGR)

    if robot:
        r, c = robot['cell']
        cx_cell = c*cw + cw//2
        cy_cell = r*ch + ch//2
        
        overlay = vis.copy()
        cv2.ellipse(overlay, (cx_cell, cy_cell), (cw//2 - 10, ch//2 - 10), 0, 0, 360, (255, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, vis, 0.5, 0, vis)
        
        yaw = robot['yaw_deg'] % 360
        angle_rad = np.radians(yaw)
        length = min(80, max(cw//2, ch//2))
        ex = int(cx_cell + length * np.sin(angle_rad))
        ey = int(cy_cell - length * np.cos(angle_rad))
        cx_px, cy_px = robot['center_px']
        cv2.arrowedLine(vis, (cx_px, cy_px), (ex, ey), (0, 255, 255), 4, tipLength=0.3)

    return vis

def send_udp_command(command):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(TIMEOUT)
        sock.sendto(command.encode(), (ESP32_IP, UDP_PORT))
        
        try:
            data, _ = sock.recvfrom(BUFFER_SIZE)
            response = data.decode().strip()
            logger.info(f"[UDP] Sent: {command} | Response: {response}")
        except:
            logger.info(f"[UDP] Sent: {command} | No response")
        
        sock.close()
        return True
    except Exception as e:
        logger.error(f"[UDP ERROR] Failed to send {command}: {e}")
        return False

selected_points = []
CARDINAL_DIRS = [(-1, 0), (0, 1), (1, 0), (0, -1)]

def mouse_callback(event, x, y, flags, param):
    global selected_points
    if event == cv2.EVENT_LBUTTONDOWN:
        r = y // CELL_SIZE_PIX[1]
        c = x // CELL_SIZE_PIX[0]
        selected_points.append((r, c))
        logger.info(f"[MOUSE] Selected {'START' if len(selected_points)==1 else 'GOAL'} -> ({r}, {c})")

def main():
    global selected_points

    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (1280, 720)}))
    picam2.start()
    time.sleep(1.0)

    last_photo_time = 0
    last_grid = None
    planner = None
    robot = None

    cmd_queue = deque()

    last_planned_pair = None

    cv2.namedWindow("D* Lite Live - Commands")
    cv2.setMouseCallback("D* Lite Live - Commands", mouse_callback)

    try:
        while True:
            frame = picam2.capture_array()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            now = time.time()

            robot = detect_robot_with_pose(gray)

            if now - last_photo_time >= PHOTO_INTERVAL:
                cv2.imwrite(os.path.join(SAVE_PHOTO_DIR, f"photo_{int(now)}.jpg"), frame)
                last_photo_time = now
                ignore = [robot['cell']] if robot else []
                new_grid, _, _ = frame_to_grid(frame, ignore_robot_cells=ignore)
                changed = grid_difference(last_grid, new_grid)
                if changed and planner:
                    planner.grid = new_grid.copy()
                    planner.update_map(changed)
                last_grid = new_grid.copy()

            if len(selected_points) >= 2 and planner is None:
                start = selected_points[0]
                goal = selected_points[1]
                grid_init, _, _ = frame_to_grid(frame)
                planner = DStarLite(grid_init, start, goal)
                planner.compute_shortest_path()
                logger.info(f"[PLANNER] Start {start} -> Goal {goal}")

            if planner and robot:
                robot_r, robot_c = robot['cell']
                planner.grid[robot['cell']] = 0
                for dr, dc in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                    nr, nc = robot_r + dr, robot_c + dc
                    if 0 <= nr < planner.rows and 0 <= nc < planner.cols:
                        planner.grid[nr, nc] = 0
                
                planner.start = robot['cell']
                planner.compute_shortest_path()

                path = planner.get_shortest_path()

                if len(path) > 1:
                    nxt = path[1]
                    curr = planner.start
                    dr = nxt[0] - curr[0]
                    dc = nxt[1] - curr[1]

                    try:
                        desired_f = CARDINAL_DIRS.index((dr, dc))
                    except ValueError:
                        desired_f = None

                    current_facing = robot['facing']
                    turn_part = []
                    if desired_f is not None:
                        diff = (desired_f - current_facing + 4) % 4
                        if diff == 1:
                            turn_part = ["RIGHT"]
                        elif diff == 3:
                            turn_part = ["LEFT"]
                        elif diff == 2:
                            turn_part = ["RIGHT", "RIGHT"]

                    ticks = int(round(CELL_SIZE_PIX[0] * METERS_PER_PIXEL / TICK_METERS))
                    forward_part = [f"FORWARD:{ticks}"]

                    combined = turn_part + forward_part
                    cmd_str = ",".join(combined) if combined else None

                    planned_pair = (curr, nxt)
                    if cmd_str and planned_pair != last_planned_pair:
                        cmd_queue.append(cmd_str)
                        last_planned_pair = planned_pair
                        nxt_cmd_to_print = cmd_queue.popleft()
                        if nxt_cmd_to_print!="Error, more than one new minima found.":
                            send_udp_command(nxt_cmd_to_print)
                        logger.info(f"[COMMAND] {nxt_cmd_to_print}")

                    if robot and 'cell' in robot:
                        planner.start = robot['cell']
                        planner.compute_shortest_path()
                    else:
                        planner.start = nxt
                        planner.compute_shortest_path()

                elif robot['cell'] == planner.goal:
                    logger.info("Goal reached!")
                    break

            vis_grid = last_grid if last_grid is not None else np.zeros((frame.shape[0]//CELL_SIZE_PIX[1], frame.shape[1]//CELL_SIZE_PIX[0]), dtype=np.uint8)
            vis = overlay_visual(frame, vis_grid, CELL_SIZE_PIX,
                                 path=planner.get_shortest_path() if planner else None,
                                 robot=robot)

            for i, (r, c) in enumerate(selected_points[:2]):
                color = (0, 255, 255) if i == 0 else (0, 165, 255)
                label = "START" if i == 0 else "GOAL"
                cv2.rectangle(vis, (c*CELL_SIZE_PIX[0], r*CELL_SIZE_PIX[1]),
                              ((c+1)*CELL_SIZE_PIX[0], (r+1)*CELL_SIZE_PIX[1]), color, 3)
                cv2.putText(vis, label, (c*CELL_SIZE_PIX[0] + 5, r*CELL_SIZE_PIX[1] + 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

            cv2.imshow("D* Lite Live - Commands", vis)
            cv2.imwrite(VIS_SAVE_FILE, vis)

            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
            if k == ord('r'):
                selected_points = []
                planner = None
                last_grid = None
                cmd_queue.clear()
                last_planned_pair = None
                logger.info("=== RESET ===")

    except KeyboardInterrupt:
        logger.info("\nStopped")
        pass
    finally:
        try:
            picam2.stop()
        except:
            pass
        cv2.destroyAllWindows()
        logger.info("Clean exit")

if __name__ == "__main__":
    main()
