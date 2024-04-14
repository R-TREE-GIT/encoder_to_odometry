#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import glob
import os
import json
from tqdm import tqdm

#path = "./dataset"
path = "./"
#annot_path = os.path.join(path,"annotations")
#img_path = os.path.join(path,"images")
#label_path = os.path.join(path,"labels")
annot_path = os.path.join(path,"annotations")
img_path = os.path.join(path,"ImageSets")
label_path = os.path.join(path,"labels")


# xml bbox 형식을 yolo bbox 형태로 변환하는 함수

def xml_to_yolo_bbox(bbox, w, h):
    # xmin, ymin, xmax, ymax
    x_center = ((bbox[2] + bbox[0]) / 2) / w
    y_center = ((bbox[3] + bbox[1]) / 2) / h
    width = (bbox[2] - bbox[0]) / w
    height = (bbox[3] - bbox[1]) / h
    return [x_center, y_center, width, height]
    
classes = []



files = glob.glob(os.path.join(annot_path, '*.xml'))
for fil in tqdm(files):
    
    basename = os.path.basename(fil)
    filename = os.path.splitext(basename)[0]
    
    result = []
    
    tree = ET.parse(fil)
    root = tree.getroot()
    width = int(root.find("size").find("width").text)
    height = int(root.find("size").find("height").text)
    for obj in root.findall('object'):
        label = obj.find("name").text
        if label not in classes:
            classes.append(label)
        index = classes.index(label)
        pil_bbox = [int(x.text) for x in obj.find("bndbox")]
        yolo_bbox = xml_to_yolo_bbox(pil_bbox, width, height)
        bbox_string = " ".join([str(x) for x in yolo_bbox])
        result.append(f"{index} {bbox_string}")
    if result:
        with open(os.path.join(label_path, f"{filename}.txt"), "w", encoding="utf-8") as f:
            f.write("\n".join(result))
