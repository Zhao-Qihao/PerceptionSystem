from onnx import load_model, save_model
from onnxconverter_common.float16 import convert_float_to_float16

# 加载原始FP32模型
model = load_model("/home/itachi/project/catkin_ws_test/src/ros2_vision_inference/models/dla34_deform_576_768_simplified.onnx")

# 转换为FP16模型
model_fp16 = convert_float_to_float16(model)

# 保存FP16模型
save_model(model_fp16, "model_fp16.onnx")