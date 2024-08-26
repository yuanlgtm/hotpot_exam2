from openvino.runtime import Core

# ��������
ie = Core()
# ��ȡonnxģ��
model = ie.read_model(model="best.onnx")
# ����ģ�͵�CPU
compiled_model = ie.compile_model(model=model, device_name="CPU")
# ��ȡģ�͵������
input_layer = compiled_model.input(0)

# ��ȡ����
input_data = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
# ִ������
results = compiled_model([input_data])
# ������
output_layer = compiled_model.output(0)
output_data = results[output_layer]