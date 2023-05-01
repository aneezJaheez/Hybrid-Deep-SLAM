#!/usr/bin/env python
import time
import argparse
import cv2
import os
import sys
import platform
import numpy as np
import yaml
from glob import glob
np.seterr(all="ignore")

from pathlib import Path
import torch
import torch.backends.cudnn as cudnn

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # yolov5 strongsort root directory
WEIGHTS = ROOT / 'weights'

if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
if str(ROOT / 'yolov8') not in sys.path:
    sys.path.append(str(ROOT / 'yolov8'))  # add yolov5 ROOT to PATH
if str(ROOT / 'trackers' / 'strongsort') not in sys.path:
    sys.path.append(str(ROOT / 'trackers' / 'strongsort'))  # add strong_sort ROOT to PATH

ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

import logging
from yolov8.ultralytics.nn.autobackend import AutoBackend
from yolov8.ultralytics.yolo.data.dataloaders.stream_loaders import LoadImages, LoadStreams
from yolov8.ultralytics.yolo.data.utils import IMG_FORMATS, VID_FORMATS
from yolov8.ultralytics.yolo.utils import DEFAULT_CFG, LOGGER, SETTINGS, callbacks, colorstr, ops
from yolov8.ultralytics.yolo.utils.checks import check_file, check_imgsz, check_imshow, print_args, check_requirements
from yolov8.ultralytics.yolo.utils.files import increment_path
from yolov8.ultralytics.yolo.utils.torch_utils import select_device
from yolov8.ultralytics.yolo.utils.ops import Profile, non_max_suppression, scale_boxes, process_mask, process_mask_native
from yolov8.ultralytics.yolo.utils.plotting import Annotator, colors, save_one_box

from trackers.multi_tracker_zoo import create_tracker
from depth import Deproject
from letterbox import LetterBox

import rospy

from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension
from hybrid_slam.msg import stereo_img, array_msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

cv2.namedWindow("Deep-Perception", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
cv2.resizeWindow("Deep-Perception", 1241, 376)

class Perception:
    def __init__(self):

        with open("/home/aneezahm001/Desktop/hybrid_slam/src/hybrid_slam/configs/configs_DeepPerception.yaml", 'r') as file:
            params = yaml.safe_load(file)

        yolo_weights = WEIGHTS / params["yolo_weights"]
        reid_weights=WEIGHTS / params["reid_weights"]

        self.tracking_method = params["tracking_method"]
        self.tracking_config = ROOT / 'trackers' / self.tracking_method / 'configs' / (self.tracking_method + '.yaml')
        #self.imgsz = (640, 640)
        self.imgsz = [int(params["imgsz"])] * 2
        self.conf_thres = float(params["conf_thres"])
        self.iou_thres = float(params["iou_thres"])
        self.max_det = int(params["max_det"])
        self.device = params["device"]
        self.show_vid = bool(params["show_vid"])
        self.save_txt = bool(params["save_txt"])
        self.save_vid = bool(params["save_vid"])
        self.classes = None
        self.agnostic_nms = bool(params["agnostic_nms"])
        self.update = bool(params["update"])
        self.project = ROOT / 'runs' / 'track'
        self.name = params["name"]
        self.exist_ok = bool(params["exist_ok"])
        self.line_thickness = int(params["line_thickness"])
        self.half = bool(params["half"])
        self.dnn = bool(params["dnn"])
        self.vid_stride = int(params["vid_stride"])
        self.use_velodyne = bool(params["use_velodyne"])
        velo_bin_dir = params["velo_bin_dir"]
        cam_calib_file = params["cam_calib_file"]
        velo_calib_file = params["velo_calib_file"]

        # Directories
        if not isinstance(yolo_weights, list):  # single yolo model
            exp_name = yolo_weights.stem
        elif type(yolo_weights) is list and len(yolo_weights) == 1:  # single models after --yolo_weights
            exp_name = Path(yolo_weights[0]).stem
        else:  # multiple models after --yolo_weights
            exp_name = 'ensemble'
    
        self.exp_name = self.name if self.name else exp_name + "_" + reid_weights.stem
        self.save_dir = increment_path(Path(self.project) / exp_name, exist_ok=self.exist_ok)  # increment run
        (self.save_dir / 'tracks' if self.save_txt else self.save_dir).mkdir(parents=True, exist_ok=True)  # make dir

        # Load model
        self.device = select_device(self.device)
        self.is_seg = '-seg' in str(yolo_weights)
        self.model = AutoBackend(yolo_weights, device=self.device, dnn=self.dnn, fp16=self.half)
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        self.imgsz = check_imgsz(self.imgsz, stride=self.stride)  # check image size

        # Dataloader
        self.bs = 1

        self.vid_path, self.vid_writer, self.txt_path = [None] * self.bs, [None] * self.bs, [None] * self.bs
        self.model.warmup(imgsz=(1 if self.pt or self.model.triton else self.bs, 3, *self.imgsz))  # warmup

        # Create as many strong sort instances as there are video sources
        self.tracker_list = []
        for i in range(self.bs):
            tracker = create_tracker(self.tracking_method, self.tracking_config, reid_weights, self.device, self.half)
            self.tracker_list.append(tracker, )
            if hasattr(self.tracker_list[i], 'model'):
                if hasattr(self.tracker_list[i].model, 'warmup'):
                    self.tracker_list[i].model.warmup()
        self.outputs = [None] * self.bs

        # Run tracking
        self.seen, self.windows, self.dt = 0, [], (Profile(), Profile(), Profile(), Profile())
        self.curr_frames, self.prev_frames = [None] * self.bs, [None] * self.bs

        #Prepare 3d localization object
        self.deprojector = Deproject(
            calib_file = cam_calib_file,
            num_disparities=96,
            block_size=9,
            window_size=11,
            use_velodyne=self.use_velodyne,
            calib_file_velo=velo_calib_file,
        )

        if self.use_velodyne:
            self.bin_paths = sorted(glob(velo_bin_dir + "*.bin"))
 
        self.pub = rospy.Publisher("semantic_percepts", array_msg, queue_size=1)
        rospy.sleep(1)

    def image_transforms(self, im):
        im = LetterBox(self.imgsz, self.pt, self.vid_stride)(image=im)
        im = im.transpose((2, 0, 1))[::-1]
        im = np.ascontiguousarray(im)
        return im

    def detect(self, message, im0s_l, im0s_r):
        im_l, im_r = self.image_transforms(im0s_l), self.image_transforms(im0s_r)

        path_l = message.path
        frame_idx = int(message.left_image.header.frame_id)

        if self.use_velodyne:
            lidar_path = self.bin_paths[frame_idx]
            self.deprojector.update_velo_camera(lidar_path, im0s_l)
        else:
            self.deprojector.compute_sgbm_disparity(
                left_image=im0s_l,
                right_image=im0s_r,
            )
            self.deprojector.update_depth_map()

        with self.dt[0]:
            im = torch.from_numpy(im_l).to(self.device)
            im = im.half() if self.half else im.float()
            im /= 255.0

            if len(im.shape) == 3:
                im = im[None]

        with self.dt[1]:
            preds = self.model(im, augment=False, visualize=False)

        with self.dt[2]:
            if self.is_seg:
                masks = []
                p = non_max_suppression(preds[0], self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det, nm=32)
                proto = preds[1][-1]
            else:
                p = non_max_suppression(preds, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)

        
        publish_data = []
        # Process detections on left image
        for i, det in enumerate(p):  # detections per image
            self.seen += 1
            
            p, im0 = path_l, im0s_l.copy()
            p = Path(p)  # to Path
            # video file
            #if source_left.endswith(VID_FORMATS):
            #    txt_file_name = p.stem
            #    save_path = str(self.save_dir / p.name)  # im.jpg, vid.mp4, ...
            # folder with imgs
            #else:
            txt_file_name = p.parent.name  # get folder name containing current img
            save_path = str(self.save_dir / p.parent.name)  # im.jpg, vid.mp4, ...
            
            self.curr_frames[i] = im0

            txt_path = str(self.save_dir / 'tracks' / txt_file_name)  # im.txt
            s_l = ""
            s_l += '%gx%g ' % im.shape[2:]  # print string
            

            annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
            
            if hasattr(self.tracker_list[i], 'tracker') and hasattr(self.tracker_list[i].tracker, 'camera_update'):
                if self.prev_frames[i] is not None and self.curr_frames[i] is not None:  # camera motion compensation
                    self.tracker_list[i].tracker.camera_update(self.prev_frames[i], self.curr_frames[i])
            
            if det is not None and len(det):
                if self.is_seg:
                    shape = im0.shape
                    # scale bbox first the crop masks
                    masks.append(process_mask(proto[i], det[:, 6:], det[:, :4], im.shape[2:], upsample=True))  # HWC
                    det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], shape).round()  # rescale boxes to im0 size
                else:
                    det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()  # rescale boxes to im0 size

                # Print results
                for c in det[:, 5].unique():
                    n = (det[:, 5] == c).sum()  # detections per class
                    s_l += f"{n} {self.names[int(c)]}{'s_l' * (n > 1)}, "  # add to string

                # pass detections to strongsort
                with self.dt[3]:
                    self.outputs[i] = self.tracker_list[i].update(det.cpu(), im0)
                
                # draw boxes for visualization
                if len(self.outputs[i]) > 0:
                    
                    if self.is_seg:
                        # Mask plotting
                        annotator.masks(
                            masks[i],
                            colors=[colors(x, True) for x in det[:, 5]],
                            im_gpu=im[i]
                        )
                    
                    for j, (output) in enumerate(self.outputs[i]):
                        bbox = output[0:4]
                        id = output[4]
                        cls = output[5]
                        conf = output[6]

                        #Localize detected objects
                        if not self.use_velodyne:
                            cam_coordinates = self.deprojector.deproject_2d3d(bbox, method="median")
                        else:
                            cam_coordinates = self.deprojector.deproject_velodyne(bbox)

                        if conf > 0.6:
                            publish_data.append([frame_idx+1, id, output[0], output[1], output[2] - output[0], output[3] - output[1], conf, cam_coordinates[0], cam_coordinates[1], cam_coordinates[2], cls])
                        if self.save_txt:
                            # to MOT format
                            bbox_left = output[0]
                            bbox_top = output[1]
                            bbox_w = output[2] - output[0]
                            bbox_h = output[3] - output[1]
                            
                            #Write results to txt file
                            with open(txt_path + '.txt', 'a') as f:
                                #MOT Compliant results
                            #     f.write(('%g ' * 10 + '\n') % (frame_idx + 1, id, bbox_left,  # MOT format
                            #                                    bbox_top, bbox_w, bbox_h, -1, -1, -1, i))

                                #Results for semantic mapping
                                f.write(('%g ' * 11 + '\n') % (frame_idx + 1, id, bbox_left,  # MOT format
                                                               bbox_top, bbox_w, bbox_h, conf, cam_coordinates[0], cam_coordinates[1], cam_coordinates[2], cls))

                        if self.save_vid or self.show_vid:  # Add bbox/seg to image
                            c = int(cls)  # integer class
                            id = int(id)  # integer id
                            
                            label = f'{id} {self.names[c]} {conf:.2f}'
                            
                            #adding 3d location label
                            label += ', {0:.1f}, {1:.1f}, {2:.1f}'.format(cam_coordinates[0], cam_coordinates[1], cam_coordinates[2])

                            color = colors(c, True)
                            annotator.box_label(bbox, label, color=color)
            else:
                pass
                #tracker_list[i].tracker.pred_n_update_all_tracks()

            # Stream results
            im0 = annotator.result()
            if self.show_vid:
                if platform.system() == 'Linux' and p not in self.windows:
                    self.windows.append(p)
                cv2.imshow("Deep-Perception", im0)
                if cv2.waitKey(1) == ord('q'):  # 1 millisecond
                    exit()

            # Save results (image with detections)
            if self.save_vid:
                if self.vid_path[i] != save_path:  # new video
                    self.vid_path[i] = save_path
                    if isinstance(self.vid_writer[i], cv2.VideoWriter):
                        self.vid_writer[i].release()  # release previous video writer
                    else:  # stream
                        fps, w, h = 30, im0.shape[1], im0.shape[0]
                    save_path = str(Path(save_path).with_suffix('.mp4'))  # force *.mp4 suffix on results videos
                    self.vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                self.vid_writer[i].write(im0)

            self.prev_frames[i] = self.curr_frames[i]

            LOGGER.info(f"Deep Perception Node: Frame {frame_idx+1} {s_l}{'' if len(det) else '(no detections), '}{sum([dt.dt for dt in self.dt if hasattr(dt, 'dt')]) * 1E3:.1f}ms taken for inference.")

            self.publish_semantics(publish_data)

    def publish_semantics(self, publish_data):
        timestamp = rospy.Time.now()

        msg = array_msg()
        msg.header.stamp = timestamp
        msg.data.data = [item for sublist in publish_data for item in sublist]
        msg.data.layout.data_offset = 0
        msg.data.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        msg.data.layout.dim[0].label = "dim1"
        msg.data.layout.dim[0].size = 3
        msg.data.layout.dim[0].stride = 33
        msg.data.layout.dim[1].label = "dim2"
        msg.data.layout.dim[1].size = 11
        msg.data.layout.dim[1].stride = 11

        self.pub.publish(msg)

    def grab_image_stereo(self, msg):
        start_time = time.time()

        bridge = CvBridge()
        try:
            image_left = bridge.imgmsg_to_cv2(msg.left_image)
            image_right = bridge.imgmsg_to_cv2(msg.right_image)
            message = msg
            self.detect(message, image_left, image_right)
        except CvBridgeError as e:
            rospy.logerr("CvBridgeError {}".format(e))
            return
        
        end_time = time.time()
        execution_time_ms = (end_time - start_time) * 1000
        print("Time taken for iteration:", execution_time_ms)

    def run_perception(self):
        rospy.init_node("deep_perception", anonymous=True)
        rospy.Subscriber("sensor_stereo", stereo_img, self.grab_image_stereo)
        rospy.spin()

def main():
    check_requirements(requirements=ROOT / 'requirements.txt', exclude=('tensorboard', 'thop'))
    perception = Perception()
    perception.run_perception()


if __name__ == "__main__":
    main()
