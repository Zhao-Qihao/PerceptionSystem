#!/bin/bash


trt_version=8431


if [ ! -f "model/rpn_centerhead_sim_fp32.plan.${trt_version}" ]; then
    echo Building the model: model/rpn_centerhead_sim_fp32.plan.${trt_version}, this will take 2 minutes. Wait a moment 🤗🤗🤗~.
    trtexec --onnx=model/rpn_centerhead_sim.onnx \
        --saveEngine=model/rpn_centerhead_sim_fp32.plan.${trt_version} \
        --workspace=4096 --outputIOFormats=fp32:chw \
        --inputIOFormats=fp32:chw --verbose --dumpLayerInfo \
        --dumpProfile --separateProfileRun \
        --profilingVerbosity=detailed > model/rpn_centerhead_sim_fp32.${trt_version}.log 2>&1

    rm -rf model/rpn_centerhead_sim_fp32.plan
    dir=`pwd`
    ln -s ${dir}/model/rpn_centerhead_sim_fp32.plan.${trt_version} model/rpn_centerhead_sim_fp32.plan
else
    echo Model model/rpn_centerhead_sim_fp32.plan.${trt_version} already build 🙋🙋🙋.
fi