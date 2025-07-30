# MAKING ASYNC MULTIPROCESS VERSION
import asyncio
from concurrent.futures import ThreadPoolExecutor

# from IPython import display

import time
from lib_xml_tree import *
from lib_connection import *
from lib_robot_transformations import *
from lib_config_loader import load_config

import tensorflow as tf
import datetime
from utils import label_map_util

import json
from ifm3dpy.device import O3D
# import argparse
from pathlib import Path
import copy, socket

import cv2                #  assumed – for AprilTag detection
import numpy as np
from ifm3dpy.pcic import FrameGrabber, buffer_id
import subprocess, logging
from typing import Tuple
from calib_routine import optimize 


tf.get_logger().setLevel('ERROR')           # Suppress TensorFlow logging (2)
# Enable GPU dynamic memory allocation
gpus = tf.config.experimental.list_physical_devices('GPU')
for gpu in gpus:
    tf.config.experimental.set_memory_growth(gpu, True)





class CameraManager:
    def __init__(self, cfg, rx_queue, tx_queue):
        self.cfg = cfg     
        self._rx_q = rx_queue # robot → calib thread
        self._tx_q = tx_queue # calib thread → robot
        
        self.state = "free" # free / calibration / inference / loading_weights / loading parameters 
        self.model = None
        self.detect_fn = None
        self.mode = "inference"   # inference / calibration

        self.o3d = O3D(ip=self.cfg["SENSOR_IP"])
        self._config_path = None  # path to the settings file of current settings
        self._config = None       # current camera settings config, read from self._config_path  
        

        # prepare to work
        self.set_inference_settings()
      
    def get_state(self):
        return self.state

    def load_model(self):
        # Load saved model and build the detection function
        self.model = tf.saved_model.load(self.cfg['PATH_TO_SAVED_MODEL'])
        self.detect_fn = self.model.signatures['serving_default']

    def calib_procedure(self):

        ################################################ 
        # # what to expect in received_dict from robot
        # sent_mess_list.append({"Sen" : {'Type' : 'ServerToCamera'}})    
        # sent_mess_list.append({"WatchDog_out" : watchDog_out})  
        # sent_mess_list.append({'Frame_assigned': frame_assigned})
        # sent_mess_list.append({'Position_reached': position_reached})
        # sent_mess_list.append({'Need_cam_cal': need_cam_cal})
        # sent_mess_list.append({'Screenshot': screenshot}) 
        ################################################
        # what to send to the robot
        # msg: dict[str, object] = {
        # "Sen":        {"Type": "Camera"},
        # "XYZ1":       {"X": "0.0", "Y": "0.0", "Z": "0.0"},
        # "XYZ2":       {"X": "0.0", "Y": "0.0", "Z": "0.0"},
        # "XYZ3":       {"X": "0.0", "Y": "0.0", "Z": "0.0"},
        # "XYZ4":       {"X": "0.0", "Y": "0.0", "Z": "0.0"},
        # "CAM_CAL_RES":{"X": "0.0", "Y": "0.0", "Z": "0.0",
        #                "A": "0.0", "B": "0.0", "C": "0.0"},
        # "WatchDog_in":     watchDog_out,
        # "Frame_assign":    "0",
        # "Move_next_pt":    "0",
        # "Cam_cal_in_proc": "0",
        # "See_4_targets":   "0",
        ################################################
        try:            
            self.mode  = "calibration"
            self.set_calibration_settings()
            self.state = "busy"

            ################################################
            # calibration logic
            # data collection
            
            cam_frames_list = []
            rob_frames_list = []
            
            self.send_robot({"Cam_cal_in_proc": '1'})  # robot <-- camera_manager
            self.wait_robot("Position_reached", '1')  # robot --> camera_manager
            
            for _ in range(self.cfg["num_calib_positions"]):              
                self.send_robot({"Move_next_pt": '1'})     # robot <-- camera_manager
                self.wait_robot("Position_reached", '0') # robot --> camera_manager
                self.send_robot({"Move_next_pt": '0'})     # robot <-- camera_manager
    
                self.wait_robot("Need_cam_cal", '1')      # robot --> camera_manager
                self.wait_robot("Position_reached", '1')  # robot --> camera_manager
                
                r = self.wait_robot("DEF_RIst", None) # robot --> camera_manager
                rob_frames_list.append([float(r[key]) for key in ['X','Y','Z','A','B','C']])
                
                T, timetag = self.capture_apriltag_transform() # make a screenshot  T = np.matrix [4,4]
                rob_frames_list.append(Rotation_matrix(T, "KUKA").extract_frame().to_list())
                
            # optimization
            result_dict = optimize(cam_frames_list, rob_frames_list)
            self.send_robot({"CAM_CAL_RES": result_dict})   # robot <-- camera_manager
            self.send_robot({"Frame_assign": '1'})          # robot <-- camera_manager
            self.wait_robot("Frame_assigned", '1')          # robot --> camera_manager
            self.send_robot({"Frame_assign": '0'})          # robot <-- camera_manager
            self.wait_robot("Need_cam_cal", '0')            # robot --> camera_manager
            self.send_robot({"Cam_cal_in_proc": '0'})       # robot <-- camera_manager
            
            ################################################
            
            self.set_inference_settings()
        finally:                
            self.mode  = "inference"
            self.state = "free"

        return result_kuka_frame




# ---------- private helpers ----------
    @staticmethod
    def _load_config_file(path: Path) -> dict:
        """Load configuration from disk into a Python dict."""
        with open(path, "r", encoding="utf‑8") as f:
            return json.load(f)

    def _push_config(self) -> None:
        """Send the current `self._config` JSON to the camera."""
        self.o3d.from_json(self._config)

    def _pull_config(self) -> None:
        """Fetch the live configuration from the camera into memory (`self._config`)."""
        self._config = self.o3d.to_json()

    # ---------- public API ----------
    def get_config(self) -> dict:
        """Return a **deep copy** of the current config so the caller can’t modify it in place."""
        return copy.deepcopy(self._config)

    def save_config_to_file(self, path: str | Path | None = None) -> None:
        """Persist the working configuration to disk."""
        path = Path(path or self._config_path)
        with open(path, "w", encoding="utf‑8") as f:
            json.dump(self._config, f, indent=2)

    def update_config(self, patch: dict, push: bool = True) -> None:
        """
        Apply a *patch* to the working config and, optionally,
        push the result to the camera immediately.

        Parameters
        ----------
        patch : dict
            Keys/values to merge into the current configuration.
            Nested dictionaries are merged recursively.
        push : bool, default=True
            If *True*, call `_push_config()` after merging.
        """
        # Recursively merge nested dictionaries
        def merge(a: dict, b: dict) -> None:
            for k, v in b.items():
                if isinstance(v, dict) and isinstance(a.get(k), dict):
                    merge(a[k], v)
                else:
                    a[k] = v

        merge(self._config, patch)
        if push:
            self._push_config()

    def reload_from_device(self) -> None:
        """Sync the local copy if someone changed the camera settings directly on the device."""
        self._pull_config()

    def set_calibration_settings(self) -> None:
        """Load calibration JSON and push it to the camera."""
        self.state = 'changing settings to calibration'
        
        cfg_path = Path(self.cfg["config_path_calibration_setting"]).expanduser().resolve()
        assert cfg_path.is_file(), f"Calibration file not found: {cfg_path}"
    
        self._config_path = cfg_path                  
        self._config = self._load_config_file(cfg_path)
        self._push_config()

        wait_camera_online(self.cfg["SENSOR_IP"])  # await for camera alive again
        self.mode = "calibration"   # inference / calibration
        self.state = "free"
        print("Calibration settings applied.")
    
    def set_inference_settings(self) -> None:        
        """Load inference JSON and push it to the camera."""
        self.state = 'changing settings to inference'
        
        cfg_path = Path(self.cfg["config_path_work_inference_setting"]).expanduser().resolve()
        assert cfg_path.is_file(), f"Inference file not found: {cfg_path}"
    
        self._config_path = cfg_path                   
        self._config = self._load_config_file(cfg_path)
        self._push_config()

        wait_camera_online(self.cfg["SENSOR_IP"])  # await for camera alive again
        self.mode = "inference"   # inference / calibration
        self.state = "free"
        print("Inference settings applied.")

    @staticmethod
    def wait_camera_online(ip: str, *, timeout: float = 10.0, interval: float = 0.5) -> None:
        """
        Blocking subproced while camera is rebooting
        await for camera ping feedback of timeout
        Makes RuntimeError, if no response from camera within timeout .
        """
        t0 = time.time()
        while True:
            #   -c 1  : послать 1 пакет
            #   -W 1  : ждать ответа 1 секунду
            if subprocess.call(["ping", "-c", "1", "-W", "1", ip],
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL) == 0:
                logging.info("Camera %s is back online (%.1f s)", ip, time.time() - t0)
                return
    
            if time.time() - t0 > timeout:
                raise RuntimeError(f"Camera {ip} did not come online in {timeout} s")
    
            time.sleep(interval)


    def wait_robot(self, field: str, desired_value : bool, timeout: float | None = None):
        """field = 'Position_reached'  – safely extract specific sig from robot telegram in background process"""
        end = time.time() + timeout if timeout else None
        while True:
            try:
                msg = self._rx_q.get(timeout=timeout)
            except Empty:
                return None
            dic = extract_xml(msg)
            if field in dic and (dic[field] == desired_value or desired_value is None):
                return dic[field]
            if end:
                timeout = max(0, end - time.time())

   
    def send_robot(self, patch: dict):
        """patch = {'Cam_cal_in_proc': '1'}  – include new sig into telegram to the robot"""
        self._tx_q.put(patch)

    async def capture_apriltag_transform(
        self,
        *,
        save_raw: bool = False,
        root_dir: Path = Path(self.cfg["root_dir"]),
        timeout_ms: int = 100,
    ) -> Tuple[np.ndarray, float]:
        """
        Grab one frame by SW‑trigger, find an AprilTag, and return the 4×4
        homogeneous transform (camera → tag).
    
        Parameters
        ----------
        save_raw : bool, default=False
            If *True* the amplitude / distance / confidence / XYZ buffers are
            saved on disk under `root_dir/YYYY_MM_DD/{AM|PM}/{index}/`.
        root_dir : Path
            Top‑level folder for raw dumps (ignored when `save_raw` == False).
        timeout_ms : int
            `FrameGrabber.wait_for_frame()` timeout.
    
        Returns
        -------
        transform : np.ndarray  [4, 4]
        timestamp : float        Unix epoch (s)
    
        Raises
        ------
        RuntimeError
            When the camera is unreachable or no AprilTag is detected.
        """
    
        loop = asyncio.get_running_loop()
    
        # -----------------------------------------------------------------
        # 1. Prepare helpers that must NOT block the asyncio loop
        # -----------------------------------------------------------------
        def _ping(ip: str, tries: int = 1) -> bool:
            """True if the device replies to ICMP."""
            cmd = ["ping", "-c", str(tries), "-W", "1", ip]
            return subprocess.call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0
    
        def _open_fg() -> FrameGrabber:
            """Create and start a new frame grabber."""
            cam = self.o3d                                 # already opened in __init__
            fg_ = FrameGrabber(cam, pcic_port=self.cfg["xmlrpc_port"])
            fg_.start([
                buffer_id.AMPLITUDE_IMAGE,
                buffer_id.RADIAL_DISTANCE_IMAGE,
                buffer_id.CONFIDENCE_IMAGE,
                buffer_id.XYZ,
            ])
            return fg_
    
        def _normalize_amplitude(buf: np.ndarray) -> np.ndarray:
            """Stretch uint16 → uint8 for debugging / storage."""
            arr = buf.copy()
            arr[arr >= 16200] = 0          # device‑specific cut‑off
            arr[:, 215:] = 0              # mask right margin
            mn, mx = arr.min(), arr.max()
            return ((arr - mn) * 255 / max(mx - mn, 1)).astype(np.uint8)
    
        def _detect(buf_amp: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
            """
            AprilTag detection stub.
            Returns (corner_px[4,2], transform_cam_tag[4,4]).
            Replace this with your real detector.
            """
            # Dummy detector: always fails
            raise RuntimeError("No AprilTag detected")
    
        # -----------------------------------------------------------------
        # 2. (Re)open the frame grabber – done in a thread
        # -----------------------------------------------------------------
        fg = await loop.run_in_executor(None, _open_fg)
    
        # -----------------------------------------------------------------
        # 3. Try to trigger & grab a frame (with basic retry)
        # -----------------------------------------------------------------
        for attempt in range(2):          # two shots: first may time out
            try:
                fg.sw_trigger()
                ok, frame = fg.wait_for_frame().wait_for(timeout_ms)
            except Exception as exc:
                logging.warning("Frame grabber error: %s", exc)
                ok = False
    
            if ok:
                break
    
            # Frame failed – check if the device is alive
            ip = self.cfg["SENSOR_IP"]
            alive = await loop.run_in_executor(None, _ping, ip)
            if not alive:
                raise RuntimeError(f"Camera {ip} is unreachable (ping failed)")
            await asyncio.sleep(0.1)      # give the sensor a moment
    
        if not ok:
            raise RuntimeError("Timeout while waiting for a frame")
    
        ts_epoch = frame.timestamps().image_time_ns / 1e9
    
        # -----------------------------------------------------------------
        # 4. Process the buffers (runs in executor to keep event‑loop free)
        # -----------------------------------------------------------------
        def _post_process():
            buf_amp  = frame.get_buffer(buffer_id.AMPLITUDE_IMAGE)
            buf_dist = frame.get_buffer(buffer_id.RADIAL_DISTANCE_IMAGE)
            buf_conf = frame.get_buffer(buffer_id.CONFIDENCE_IMAGE)
            buf_xyz  = frame.get_buffer(buffer_id.XYZ)          # shape (H,W,3)
    
            amp_u8 = _normalize_amplitude(buf_amp)
    
            corners_px, T_cam_tag = _detect(amp_u8)
    
            if save_raw:
                # Build folder  …/YYYY_MM_DD/AM|PM/N/
                now       = datetime.datetime.now()
                day_dir   = root_dir / now.strftime("%Y_%m_%d") / now.strftime("%p")
                index_dir = max([int(p.name) for p in day_dir.iterdir() if p.is_dir()] + [0]) + 1
                dump_dir  = day_dir / f"{index_dir:04d}"
                dump_dir.mkdir(parents=True, exist_ok=True)
    
                base = dump_dir / now.strftime("%Y_%m_%d_%H_%M_%S")
                np.save(base.with_suffix("_xyz.npy"), buf_xyz)
                cv2.imwrite(str(base.with_suffix("_amp.png")),  amp_u8)
                cv2.imwrite(str(base.with_suffix("_dist.png")), buf_dist)
                cv2.imwrite(str(base.with_suffix("_conf.png")), buf_conf)
    
            return T_cam_tag
    
        T = await loop.run_in_executor(None, _post_process)
        return T, ts_epoch
        

async def main():
    # Load IP, PORT configuration from config
    config = load_config()
    target_address = config['clients']['camera']['target_address']  
    target_port = config['clients']['camera']['target_port']
 
    # Create socket
    UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    server_address =  (target_address, target_port)

    # CREATE CameraManager object
    rx_queue = Queue()    # robot → calib thread
    tx_queue = Queue()    # calib thread → robot
    cm = CameraManager(cfg = config['camera_manager'], rx_queue=rx_queue, tx_queue=tx_queue)
    cm.load_model()

    # PREPARE CHILDREN SUBROCESS POOL
    loop = asyncio.get_running_loop()
    executor = ThreadPoolExecutor()

    
    # PREPARE TO START LOOP
    telegram = create_xml_fast([{'IPOC': '0'}])
    print('start_mess = :', telegram)
    # init dependant telegram variables
    watchDog_out = '0'
    proc = None
    thread = None

    cfg_future: asyncio.Future | None = None     # перед while True # background process handler
    while True:
        ################################################
        # SENDING MESSAGES TO CAMERA SERVER-SUBPROCESS
        ################################################  
        TransmitData = telegram
        SendData(TransmitData, UDPClientSocket, server_address)

        


        ################################################
        # RECEIVING MESSAGES FROM CAMERA SERVER-SUBPROCESS
        ################################################ 
        bytesAddressPair = UDPClientSocket.recvfrom(4096)

        # ----------  put robot telegram into calibration stream ------------
        if cfg_future and not cfg_future.done():  # only when calibration is going on
            rx_queue.put_nowait(raw_msg)          # <── for calib thread
        # ------------------------------------------------ ------------    

        ReceivedMessage = bytesAddressPair[0]
        address = bytesAddressPair[1]
        # clientMsg = "Message from Client:{}".format(ReceivedMessage)
        # clientIP = "Client IP Address:{}".format(address)
        #print(clientMsg)
        #print(clientIP)
        received_dict = extract_xml(ReceivedMessage)
        ##print('camera_received_telegram', ReceivedMessage)

        # use received data from cam
        try:
            watchDog_out = received_dict['WatchDog_out']
        except Exception as ex:
            print(f" Data can not be extracted from received to camera telegram: {ex}")   

        ################################################
        # MAIN OPERATION LOGIC
        ################################################ 
        # RUN HEAVY TASK IN BACKGROUND MODE

        # --- Start a background switch --------------------------------
        if received_dict['need_cam_cal'] == "1" and cm.mode == "inference" and cm.state == "free" and cfg_future is None:
            cfg_future = loop.run_in_executor(None, cm.calib_procedure)
            logging.info("Started calibration")

        # --- Check if the calibration has finished -------------------------
        if cfg_future and cfg_future.done():
            try:
                calibration_frame = cfg_future.result()          # propagate exceptions
                logging.info(f"Camera calibration executed successfully, frame: {calibration_frame}")
            except Exception as exc:
                logging.error("Camera calibration failed: %s", exc)                
            finally:
                cfg_future = None            # ready for the next request


        if received_dict['need_cam_cal'] == "0" and cm.mode == "inference" and cm.state == "free":
            # DO INFERENCE. SYNC or ASYNC

        ################################################
        # END OF MAIN OPERATION LOGIC
        ################################################ 
        # # what to expect in received_dict from robot
        # sent_mess_list.append({"Sen" : {'Type' : 'ServerToCamera'}})    
        # sent_mess_list.append({"WatchDog_out" : watchDog_out})  
        # sent_mess_list.append({'Frame_assigned': frame_assigned})
        # sent_mess_list.append({'Position_reached': position_reached})
        # sent_mess_list.append({'Need_cam_cal': need_cam_cal})
        # sent_mess_list.append({'Screenshot': screenshot}) 

        ################################################
        # PREPARING THE MESSAGE TO BE SENT
        ################################################ 
        msg: dict[str, object] = {
            "Sen":        {"Type": "Camera"},
            "XYZ1":       {"X": "0.0", "Y": "0.0", "Z": "0.0"},
            "XYZ2":       {"X": "0.0", "Y": "0.0", "Z": "0.0"},
            "XYZ3":       {"X": "0.0", "Y": "0.0", "Z": "0.0"},
            "XYZ4":       {"X": "0.0", "Y": "0.0", "Z": "0.0"},
            "CAM_CAL_RES":{"X": "0.0", "Y": "0.0", "Z": "0.0",
                           "A": "0.0", "B": "0.0", "C": "0.0"},
            "WatchDog_in":     watchDog_out,
            "Frame_assign":    "0",
            "Move_next_pt":    "0",
            "Cam_cal_in_proc": "0",
            "See_4_targets":   "0",
        }
        
        # ----------  apply signals from calibration stream ------------
        if cfg_future and not cfg_future.done(): # only when calibration
            while True:
                try:
                    patch = tx_queue.get_nowait()     # patch: dict[str, Any]
                except Empty:
                    break
                msg.update(patch)                     
        
        # ----------  convert into needed format --------------------------
        sent_mess_list = [{k: v} for k, v in msg.items()]
        
        telegram = create_xml_fast(sent_mess_list)
        
                
        #display.clear_output(wait=True)
        await asyncio.sleep(0.001) # DO NOT CHANGE


    
if __name__ == '__main__':
    #await main() # TO USE IN  JUPYTER NITEBOOK ONLY
    asyncio.run(main()) # TO USE IN SCRIPT, NOT JUPYTER NITEBOOK

        
        