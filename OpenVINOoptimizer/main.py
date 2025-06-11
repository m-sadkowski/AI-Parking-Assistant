import time
import numpy as np
import onnxruntime as ort
from openvino.runtime import Core
import matplotlib.pyplot as plt
import matplotlib
# --- Przygotowanie danych wejściowych ---
# PRZED ODPALENIEM  w konsoli
# mo - -input_model FinalAgent.onnx - -output_dir optimized_model

matplotlib.use('TkAgg')
input_obs_0 = np.random.randn(1, 10).astype(np.float32)
input_obs_1 = np.random.randn(1, 17).astype(np.float32)
print("Ładowanie modelu ONNX...")
onnx_session = ort.InferenceSession("FinalAgent.onnx")

onnx_input_name0 = onnx_session.get_inputs()[0].name
onnx_input_name1 = onnx_session.get_inputs()[1].name

print (onnx_input_name1, onnx_input_name0)
for input in onnx_session.get_inputs():
    print(f"Nazwa: {input.name}, Kształt: {input.shape}, Typ: {input.type}")

print("Testowanie czasu inferencji dla ONNX...")
start_time = time.time()
for _ in range(100000):
    onnx_output = onnx_session.run(None,
                                   {onnx_input_name0 : input_obs_0,
                                           onnx_input_name1 : input_obs_1})
onnx_time = (time.time() - start_time) / 100

print(f"Średni czas inferencji (ONNX): {onnx_time:.6f} sek.")

print("Ładowanie modelu OpenVINO IR...")
core = Core()
model_ir = core.read_model("optimized_model/FinalAgent.xml")
compiled_model_ir = core.compile_model(model_ir, "CPU")
input_key = compiled_model_ir.input(0).get_any_name()

print("Testowanie czasu inferencji dla OpenVINO IR...")
start_time = time.time()
for _ in range(100000):
    result_ir = compiled_model_ir({onnx_input_name0: input_obs_0,
                                   onnx_input_name1: input_obs_1})
ir_time = (time.time() - start_time) / 100

print(f"Średni czas inferencji (OpenVINO IR): {ir_time:.6f} sek.")

print("Porównywanie wyników predykcji...")
onnx_output = onnx_output[0]
ir_output = result_ir[0]

print("ONNX wynik shape:", onnx_output.shape)
print("OpenVINO wynik shape:", ir_output.shape)
print("ONNX wynik (fragment):", onnx_output.flatten()[:10])
print("OpenVINO wynik (fragment):", ir_output.flatten()[:10])

difference = np.abs(onnx_output - ir_output)
mean_diff = np.mean(difference)
max_diff = np.max(difference)

print(f"Średnia różnica wyników: {mean_diff:.6f}")
print(f"Max różnica wyników: {max_diff:.6f}")

plt.hist(difference.flatten(), bins=50)
plt.title("Histogram różnic między ONNX a OpenVINO IR")
plt.xlabel("Różnica")
plt.ylabel("Częstość")
plt.show()

print("\nPodsumowanie:")
print(f"ONNX: Średni czas inferencji = {onnx_time:.6f} sek.")
print(f"OpenVINO IR: Średni czas inferencji = {ir_time:.6f} sek.")
print(f"Średnia różnica wyników = {mean_diff:.6f}")
print(f"Maksymalna różnica wyników = {max_diff:.6f}")