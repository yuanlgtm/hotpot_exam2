from openvino.runtime import Core

# 创建对象
ie = Core()
# 读取onnx模型
model = ie.read_model(model="best.onnx")
# 编译模型到CPU
compiled_model = ie.compile_model(model=model, device_name="CPU")
# 获取模型的输入层
input_layer = compiled_model.input(0)

# 读取数据
input_data = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
# 执行推理
results = compiled_model([input_data])
# 处理结果
output_layer = compiled_model.output(0)
output_data = results[output_layer]