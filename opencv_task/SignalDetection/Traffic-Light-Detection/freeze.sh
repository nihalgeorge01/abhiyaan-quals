#!/bin/bash
model=$1 #= faster_rcnn, ssd_inception, ssd_mobilenet
ckpt_ver=$2 # model checkpoint version num, say 10000
tf_research_path=~/Repos/abhiyaan/quals/opencv_task/SignalDetection/models/research

python $tf_research_path/object_detection/export_inference_graph.py \
    --pipeline_config_path=config/$model.config \
    --trained_checkpoint_prefix=models/training_$model/model.ckpt-$ckpt_ver \
    --output_directory=models/frozen_$model/
