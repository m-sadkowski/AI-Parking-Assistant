import time
import numpy as np
import onnxruntime as ort
from openvino.runtime import Core
import matplotlib.pyplot as plt

# --- Przygotowanie danych wejściowych ---
input_data = np.random.randn(1, 3, 224, 224).astype(np.float32)

# --- Niezoptymalizowany model (ONNX) ---
print("Ładowanie modelu ONNX...")
onnx_session = ort.InferenceSession("FinalAgent.onnx")
onnx_input_name = onnx_session.get_inputs()[0].name

print("Testowanie czasu inferencji dla ONNX...")
start_time = time.time()
for _ in range(100):  # Test dla 100 predykcji
    onnx_output = onnx_session.run(None, {onnx_input_name: input_data})
onnx_time = (time.time() - start_time) / 100  # Średni czas jednej predykcji

print(f"Średni czas inferencji (ONNX): {onnx_time:.6f} sek.")

# --- Zoptymalizowany model (OpenVINO IR) ---
print("Ładowanie modelu OpenVINO IR...")
core = Core()
model_ir = core.read_model("optimized_model/model.xml")
compiled_model_ir = core.compile_model(model_ir, "CPU")
input_key = compiled_model_ir.input(0).get_any_name()

print("Testowanie czasu inferencji dla OpenVINO IR...")
start_time = time.time()
for _ in range(100):  # Test dla 100 predykcji
    result_ir = compiled_model_ir([input_data])
ir_time = (time.time() - start_time) / 100  # Średni czas jednej predykcji

print(f"Średni czas inferencji (OpenVINO IR): {ir_time:.6f} sek.")

# --- Porównanie wyników ---
print("Porównywanie wyników predykcji...")
onnx_output = onnx_output[0]  # Wyciągnięcie wyników
ir_output = result_ir[0]

difference = np.abs(onnx_output - ir_output)
mean_diff = np.mean(difference)
max_diff = np.max(difference)

print(f"Średnia różnica wyników: {mean_diff:.6f}")
print(f"Max różnica wyników: {max_diff:.6f}")

# --- Wizualizacja różnic ---
plt.hist(difference.flatten(), bins=50)
plt.title("Histogram różnic między ONNX a OpenVINO IR")
plt.xlabel("Różnica")
plt.ylabel("Częstość")
plt.show()

# --- Podsumowanie ---
print("\nPodsumowanie:")
print(f"ONNX: Średni czas inferencji = {onnx_time:.6f} sek.")
print(f"OpenVINO IR: Średni czas inferencji = {ir_time:.6f} sek.")
print(f"Średnia różnica wyników = {mean_diff:.6f}")
print(f"Maksymalna różnica wyników = {max_diff:.6f}")
