import os
import asyncio
from concurrent.futures import ThreadPoolExecutor

import time
import json
import copy
import socket
import datetime
import subprocess
import logging

from pathlib import Path
from typing import Tuple

import cv2
import numpy as np
import tensorflow as tf

from ifm3dpy.device import O3D
from ifm3dpy.framegrabber import FrameGrabber, buffer_id

from queue import Queue, Empty

from lib_xml_tree import *
from lib_connection import *
from lib_robot_transformations import *
from lib_config_loader import load_config
from calib_routine import optimize

from argparse import ArgumentParser
from apriltag.scripts import apriltag


tf.get_logger().setLevel("ERROR")

gpus = tf.config.experimental.list_physical_devices("GPU")
for gpu in gpus:
    tf.config.experimental.set_memory_growth(gpu, True)


class CameraManager:
    def __init__(self, cfg, rx_queue, tx_queue):
        self.cfg = cfg
        self._rx_q = rx_queue
        self._tx_q = tx_queue
        self._robot_state = {}

        self.state = "free"
        self.mode = "inference"

        self.model = None
        self.detect_fn = None

        self.o3d = O3D(ip=self.cfg["SENSOR_IP"])
        self._config_path = None
        self._config = None

        self.set_inference_settings()

    def get_state(self):
        return self.state

    def load_model(self):
        self.model = tf.saved_model.load(self.cfg["PATH_TO_SAVED_MODEL"])
        self.detect_fn = self.model.signatures["serving_default"]

    def calib_procedure(self):
        print("Calibration subroutine started")
        result_dict = None

        try:
            self.state = "busy"
            self.mode = "calibration"
            self.set_calibration_settings()

            cam_frames_list = []
            rob_frames_list = []

            self.send_robot({"Cam_cal_in_proc": "1"})
            print("sent to robot Cam_cal_in_proc 1")
            
            print("wait for Position_reached 1")
            r = self.wait_robot("Position_reached", "1", timeout=100000.0)
            if r is None:
                raise RuntimeError("Timeout waiting for Position_reached=1 before calibration start")

            for i in range(self.cfg["num_calib_positions"]):
                print(f"[Calibration] Step {i + 1}/{self.cfg['num_calib_positions']}")

                self.send_robot({"Move_next_pt": "1"})
                print("sent to robot Move_next_pt 1")
                
                print("wait for Position_reached 0")
                r = self.wait_robot("Position_reached", "0", timeout=100000.0)
                if r is None:
                    raise RuntimeError("Timeout waiting for Position_reached=0 after Move_next_pt=1")

                self.send_robot({"Move_next_pt": "0"})
                print("sent to robot Move_next_pt 0")

                print("wait for Need_cam_cal 1")
                r = self.wait_robot("Need_cam_cal", "1", timeout=100000.0)
                if r is None:
                    raise RuntimeError("Timeout waiting for Need_cam_cal=1 during calibration")

                print("wait for Position_reached 1")
                r = self.wait_robot("Position_reached", "1", timeout=100000.0)
                if r is None:
                    raise RuntimeError("Timeout waiting for Position_reached=1 at calibration point")


                print("wait for RIst data")
                robot_pose = self.wait_robot("RIst", None, timeout=100000.0)
                
                if robot_pose is None:
                    raise RuntimeError("Timeout waiting for RIst from robot")
                if not isinstance(robot_pose, dict):
                    raise RuntimeError(f"Invalid RIst payload: {robot_pose}")

                
                rob_frames_list.append([
                    float(robot_pose[key]) for key in ["X", "Y", "Z", "A", "B", "C"]
                ])
                print("saved robot position")

                T = self.capture_apriltag_transform()
                if T is None:
                    rob_frames_list.pop() # remove last robot position
                    print('popped last robot pose due to no camera detecions')
                else:    
                    cam_frames_list.append(
                        Rotation_matrix(T, "KUKA").extract_frame().to_list()
                    )
                    print('Saved camera detection')
                
                assert len(rob_frames_list) == len(cam_frames_list), "Collected robot positions as not equal to collected camera detections"

            result_dict = optimize(cam_frames_list, rob_frames_list)

            self.send_robot({"CAM_CAL_RES": result_dict})
            print("sent to the robot result ")
            self.send_robot({"Frame_assign": "1"})
            print('sent to robot Frame_assign 1')

            print("wait for Frame_assigned 1")
            r = self.wait_robot("Frame_assigned", "1", timeout=100000.0)
            if r is None:
                raise RuntimeError("Timeout waiting for Frame_assigned=1")

            self.send_robot({"Frame_assign": "0"})
            print("sent to the robot Frame_assign 0 ")

            print("wait for Need_cam_cal 0")
            r = self.wait_robot("Need_cam_cal", "0", timeout=100000.0)
            if r is None:
                raise RuntimeError("Timeout waiting for Need_cam_cal=0 after calibration")

            self.send_robot({"Cam_cal_in_proc": "0"})
            print("sent to the robot Cam_cal_in_proc 0 ")

            return result_dict

        except Exception as e:
            print(f"[Client] Calibration has fallen down: {e}")
            raise

        finally:
            try:
                self.set_inference_settings()
            except Exception as e:
                print(f"[Client] Failed to restore inference settings: {e}")

            self.mode = "inference"
            self.state = "free"

    @staticmethod
    def _load_config_file(path: Path) -> dict:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)

    def _push_config(self) -> None:
        pass
        #self.o3d.from_json(self._config)

    def _pull_config(self) -> None:
        pass
        #self._config = self.o3d.to_json()

    def get_config(self) -> dict:
        return copy.deepcopy(self._config)

    def save_config_to_file(self, path=None) -> None:
        path = Path(path or self._config_path)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self._config, f, indent=2)

    def update_config(self, patch: dict, push: bool = True) -> None:
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
        self._pull_config()

    def set_calibration_settings(self) -> None:
        self.state = "changing settings to calibration"

        cfg_path = Path(self.cfg["config_path_calibration_setting"]).expanduser().resolve()
        assert cfg_path.is_file(), f"Calibration file not found: {cfg_path}"

        self._config_path = cfg_path
        self._config = self._load_config_file(self._config_path)

        # новое соединение перед push
        self.o3d = O3D(ip=self.cfg["SENSOR_IP"])
        self._push_config()

        self.wait_camera_online(self.cfg["SENSOR_IP"])
        time.sleep(2.0)

        # после reboot/reconnect лучше создать новый O3D ещё раз
        self.o3d = O3D(ip=self.cfg["SENSOR_IP"])

        self.mode = "calibration"
        self.state = "free"
        print("Calibration settings applied.")

    def set_inference_settings(self) -> None:
        self.state = "changing settings to inference"

        cfg_path = Path(self.cfg["config_path_work_inference_setting"]).expanduser().resolve()
        assert cfg_path.is_file(), f"Inference file not found: {cfg_path}"

        self._config_path = cfg_path
        self._config = self._load_config_file(self._config_path)

        self.o3d = O3D(ip=self.cfg["SENSOR_IP"])
        self._push_config()

        self.wait_camera_online(self.cfg["SENSOR_IP"])
        time.sleep(2.0)

        self.o3d = O3D(ip=self.cfg["SENSOR_IP"])

        self.mode = "inference"
        self.state = "free"
        print("Inference settings applied.")

    @staticmethod
    def wait_camera_online(ip: str, *, timeout: float = 10.0, interval: float = 0.5) -> None:
        t0 = time.time()
        while True:
            if subprocess.call(
                ["ping", "-c", "1", "-W", "1", ip],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            ) == 0:
                logging.info("Camera %s is back online (%.1f s)", ip, time.time() - t0)
                return

            if time.time() - t0 > timeout:
                raise RuntimeError(f"Camera {ip} did not come online in {timeout} s")

            time.sleep(interval)

    def wait_robot(self, field, desired_value=None, timeout=None):
        end = time.time() + timeout if timeout is not None else None
        
        #print(f"desired_value {desired_value} {type(desired_value)}") 

        # Сначала проверяем уже накопленное состояние
        if field in self._robot_state:
            value = self._robot_state[field]
            #print(f"value3 {value} {type(value)}") 
            if desired_value is None or value == desired_value:
                #print(f"value2 {value} {type(value)}") 
                return value
         
        #print("entering loop") 
        while True:
            if end is None:
                current_timeout = None
            else:
                current_timeout = end - time.time()
                if current_timeout <= 0:
                    return None

            try:
                raw_msg = self._rx_q.get(timeout=current_timeout)
            except Empty:
                return None

            dic = extract_xml(raw_msg)
            #print(f"raw msg: {dic}")

            # Сохраняем всё, что пришло, чтобы ничего не потерять
            for key, value in dic.items():
                if '.' in value:
                    value = value[ : value.index('.')]
                self._robot_state[key] = value
                
            #print(f"rob state dict: {self._robot_state}")    

            # Если в текущем сообщении есть нужное поле — возвращаем
            #if field in dic:
            #    value = dic[field]
            #    if desired_value is None or value == desired_value:
            #        print(f"value1 {value} {type(value)}") 
            #        return value

            # Либо оно уже накопилось ранее
            if field in self._robot_state:
                value = self._robot_state[field]
                if desired_value is None or value == desired_value:
                    #print(f"value0 {value} {type(value)}") 
                    return value

    def send_robot(self, patch: dict):
        self._tx_q.put(patch)

    def capture_apriltag_transform(
        self,
        *,
        save_raw: bool = False,
        root_dir: Path = None,
        timeout_ms: int = 300,
        retry_count: int = 10,
    ) -> np.ndarray:
        if root_dir is None:
            root_dir = Path(self.cfg["root_dir"])

        def _open_fg() -> FrameGrabber:
            fg = FrameGrabber(self.o3d, pcic_port=self.cfg["xmlrpc_port"])
            fg.start([
                buffer_id.NORM_AMPLITUDE_IMAGE,
                buffer_id.RADIAL_DISTANCE_IMAGE,
                buffer_id.CONFIDENCE_IMAGE,
                buffer_id.XYZ,
            ])
            return fg

        detector = apriltag.Detector(searchpath=apriltag._get_dll_path())
    
        fg = None
        try:
            fg = _open_fg()
    
            for attempt in range(retry_count):
                try:
                    #fg.sw_trigger()
                    ok, frame = fg.wait_for_frame().wait_for(timeout_ms)
                except Exception as exc:
                    print(f"[Camera] Frame error on attempt {attempt + 1}: {exc}")
                    ok, frame = False, None

                if not ok or frame is None:
                    time.sleep(0.1)
                    continue

                amp = frame.get_buffer(buffer_id.NORM_AMPLITUDE_IMAGE).copy()
                pt = frame.get_buffer(buffer_id.XYZ)

                rows = amp.shape[1]
                cols = amp.shape[0]

                amp = amp.copy()
                amp[amp >= 3000] = 3000
                if cols > 245:
                    amp[245:, :] = 0

                frame1 = cv2.normalize(amp, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

                result, overlay = apriltag.detect_tags(
                    frame1,
                    detector,
                    camera_params=(3156.71852, 3129.52243, 359.097908, 239.736909),
                    tag_size=0.2,
                    vizualization=3,
                    verbose=0,
                    annotation=1.0,
                )

                if result is not None and len(result) >= 2:


                    tag = result[0]
                    T_mtx = result[1]

                    sumx = 0.0
                    sumy = 0.0
                    sumz = 0.0
                    crnrs = 0

                    for corner in tag.corners:
                        x = pt[int(corner[1]), int(corner[0])][0]
                        y = pt[int(corner[1]), int(corner[0])][1]
                        z = pt[int(corner[1]), int(corner[0])][2]
                        if x != 0 or y != 0 or z != 0:
                            sumx += x
                            sumy += y
                            sumz += z
                            crnrs += 1

                    if crnrs == 4:
                        centroid = (sumx / 4.0, sumy / 4.0, sumz / 4.0)
                        T_mtx[:3, -1] = centroid
    
                        T_cam_tag = np.asarray(T_mtx, dtype=np.float64)
                        if T_cam_tag.shape != (4, 4):
                            raise RuntimeError(f"Invalid transform shape: {T_cam_tag.shape}")

                        print(f"Detected matrix: {T_cam_tag}")
                        return T_cam_tag

                time.sleep(0.05)

            return None

        finally:
            if fg is not None:
                try:
                    fg.stop()
                except Exception:
                    pass
                    


def apply_pending_tx_patches(tx_queue: Queue, msg: dict) -> None:
    """Read and apply all pending patches from calibration thread."""
    while True:
        try:
            patch = tx_queue.get_nowait()
            msg.update(patch)
        except Empty:
            break


def build_telegram(msg: dict) -> bytes:
    """Convert current message dict into XML telegram bytes."""
    sent_mess_list = [{k: v} for k, v in msg.items()]
    return create_xml_fast(sent_mess_list)


def send_current_telegram(udp_client_socket: socket.socket, server_address, msg: dict) -> None:
    """Build and send one current telegram to server."""
    telegram = build_telegram(msg)
    SendData(telegram, udp_client_socket, server_address)


def handle_received_message(
    received_message: bytes,
    msg: dict,
    rx_queue: Queue,
    cfg_future,
    last_need_cam_cal: int,
    cm: CameraManager,
) -> int:
    try:
        received_dict = extract_xml(received_message)
    except Exception as ex:
        print(f"[Client] Failed to parse XML: {ex}")
        return last_need_cam_cal

    #print("[RX parsed]:", received_dict)

    # всегда сохраняем последнее состояние робота
    for key, value in received_dict.items():
        cm._robot_state[key] = value

    # во время активной калибровки форвардим весь пакет в очередь
    if cfg_future is not None and not cfg_future.done():
        try:
            rx_queue.put_nowait(received_message)
            #print("[RX->CALIB] forwarded packet")
        except Exception as ex:
            print(f"[Client] Failed to push message to rx_queue: {ex}")

    try:
        msg["WatchDog_in"] = received_dict["WatchDog_out"]
    except Exception as ex:
        print(f"WatchDog_out Data can not be extracted from received telegram: {ex}")

    try:
        return int(float(received_dict.get("Need_cam_cal", 0)))
    except Exception as ex:
        print(f"need_cam_cal forced to previous value: {ex}")
        return last_need_cam_cal


def receive_all_available_packets(
    udp_client_socket: socket.socket,
    msg: dict,
    rx_queue: Queue,
    cfg_future,
    last_need_cam_cal: int,
    cm: CameraManager,
) -> int:
    need_cam_cal = last_need_cam_cal

    while True:
        try:
            received_message, address = udp_client_socket.recvfrom(4096)
        except BlockingIOError:
            break
        except Exception as ex:
            print(f"[Client] recvfrom failed: {ex}")
            break

        need_cam_cal = handle_received_message(
            received_message=received_message,
            msg=msg,
            rx_queue=rx_queue,
            cfg_future=cfg_future,
            last_need_cam_cal=need_cam_cal,
            cm=cm,
        )

    return need_cam_cal


def maybe_start_calibration(loop, executor, cm: CameraManager, cfg_future, need_cam_cal: int):
    """Start background calibration if requested and if no calibration is running."""
    if (
        need_cam_cal == 1
        and cfg_future is None
        and cm.mode == "inference"
        and cm.state == "free"
    ):
        print("Calibration started")
        logging.info("Started calibration")
        return loop.run_in_executor(executor, cm.calib_procedure)

    return cfg_future


def handle_finished_calibration(cfg_future):
    """Check whether background calibration has finished and finalize its future."""
    if cfg_future is not None and cfg_future.done():
        try:
            calibration_result = cfg_future.result()
            logging.info("Camera calibration executed successfully, frame: %s", calibration_result)
        except Exception as exc:
            logging.error("Camera calibration failed: %s", exc)
        return None

    return cfg_future


async def main():
    config = load_config()
    target_address = config["clients"]["camera"]["target_address"]
    target_port = config["clients"]["camera"]["target_port"]

    udp_client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_client_socket.setblocking(False)
    server_address = (target_address, target_port)

    rx_queue = Queue() # from robot to calibation
    tx_queue = Queue() # fro, calibration to robot

    cm = CameraManager(
        cfg=config["camera_manager"],
        rx_queue=rx_queue, 
        tx_queue=tx_queue,
    )
    cm.load_model()

    loop = asyncio.get_running_loop()
    executor = ThreadPoolExecutor(max_workers=2)

    # template mgg camera -> robot
    msg: dict[str, object] = {
        "Sen": {"Type": "Camera"},
        "XYZ1": {"X": "0.0", "Y": "0.0", "Z": "0.0"},
        "XYZ2": {"X": "0.0", "Y": "0.0", "Z": "0.0"},
        "XYZ3": {"X": "0.0", "Y": "0.0", "Z": "0.0"},
        "XYZ4": {"X": "0.0", "Y": "0.0", "Z": "0.0"},
        "CAM_CAL_RES": {"X": "0.0", "Y": "0.0", "Z": "0.0", "A": "0.0", "B": "0.0", "C": "0.0"},
        "WatchDog_in": "0",
        "Frame_assign": "0",
        "Move_next_pt": "0",
        "Cam_cal_in_proc": "0",
        "See_4_targets": "0",
    }

    cfg_future = None
    need_cam_cal = 0

    print("Started loop in camera client...")

    try:
        while True:
            apply_pending_tx_patches(tx_queue, msg)
            try:
                send_current_telegram(udp_client_socket, server_address, msg)
            except Exception as ex:
                print(f"[Client] SendData failed: {ex}")

            need_cam_cal = receive_all_available_packets(
                udp_client_socket=udp_client_socket,
                msg=msg,
                rx_queue=rx_queue,
                cfg_future=cfg_future,
                last_need_cam_cal=need_cam_cal,
                cm=cm,
            )

            cfg_future = maybe_start_calibration(
                loop=loop,
                executor=executor,
                cm=cm,
                cfg_future=cfg_future,
                need_cam_cal=need_cam_cal,
            )

            cfg_future = handle_finished_calibration(cfg_future)

            if need_cam_cal == 0 and cm.mode == "inference" and cm.state == "free":
                pass
                # inference logic here

            await asyncio.sleep(0.001)

    finally:
        executor.shutdown(wait=False, cancel_futures=True)
        udp_client_socket.close()


if __name__ == "__main__":
    asyncio.run(main())
