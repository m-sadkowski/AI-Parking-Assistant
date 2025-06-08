import re
import matplotlib.pyplot as plt
import numpy as np

"""
Mean Reward (Średnia nagroda) - to średnia wartość nagród otrzymywanych przez model 
w danym etapie treningu. Wzrost tej wartości oznacza, że model uczy się podejmować 
lepsze decyzje. W idealnym przypadku powinniśmy obserwować stopniowy wzrost tej wartości.

Std of Reward (Odchylenie standardowe nagrody) - mierzy zmienność otrzymywanych nagród. 
Wysokie odchylenie oznacza, że model czasem otrzymuje bardzo dobre, a czasem bardzo złe wyniki.
Pożądane jest stopniowe zmniejszanie się tej wartości wraz ze stabilizacją działania modelu.
"""

steps = []
mean_rewards = []
std_rewards = []
time_elapsed = []

pattern = re.compile(
    r'Step: (\d+)\. Time Elapsed: ([\d.]+) s\. Mean Reward: ([\d.-]+)\. Std of Reward: ([\d.-]+)\.')

with open('results.txt', 'r') as file:
    for line in file:
        match = pattern.search(line)
        if match:
            steps.append(int(match.group(1)))
            time_elapsed.append(float(match.group(2)))
            mean_rewards.append(float(match.group(3)))
            std_rewards.append(float(match.group(4)))

steps = np.array(steps)
mean_rewards = np.array(mean_rewards)
std_rewards = np.array(std_rewards)
time_elapsed = np.array(time_elapsed)

median_mean_reward = np.median(mean_rewards)
average_mean_reward = np.mean(mean_rewards)

time_per_10k_steps = np.diff(time_elapsed)[::1]
average_time_per_10k = np.mean(time_per_10k_steps)

plt.figure(figsize=(14, 6))

plt.subplot(1, 2, 1)
plt.plot(steps, mean_rewards, label='Średnia nagroda', color='blue', linewidth=2)
plt.axhline(y=median_mean_reward, color='red', linestyle='--',
            label=f'Mediana: {median_mean_reward:.2f}')
plt.axhline(y=average_mean_reward, color='green', linestyle='--',
            label=f'Średnia: {average_mean_reward:.2f}')
plt.xlabel('Liczba kroków treningowych', fontsize=12)
plt.ylabel('Wartość średniej nagrody', fontsize=12)
plt.title('Zmiany średniej nagrody w trakcie treningu', fontsize=14, pad=20)
plt.legend(fontsize=10, loc='lower right')
plt.grid(True, linestyle='--', alpha=0.7)

plt.subplot(1, 2, 2)
plt.plot(steps, std_rewards, label='Odchylenie standardowe', color='orange', linewidth=2)
plt.xlabel('Liczba kroków treningowych', fontsize=12)
plt.ylabel('Wartość odchylenia standardowego', fontsize=12)
plt.title('Zmiany odchylenia standardowego nagrody', fontsize=14, pad=20)
plt.legend(fontsize=10, loc='upper right')
plt.grid(True, linestyle='--', alpha=0.7)

plt.tight_layout()
plt.savefig("wykres1.png")
plt.show()



print("\nPODSUMOWANIE STATYSTYK TRENINGU:")
print(f"- Średnia wartość nagrody w całym treningu: {average_mean_reward:.3f}")
print(f"- Mediana wartości nagród: {median_mean_reward:.3f}")
print(f"- Średni czas wykonania 10,000 kroków: {average_time_per_10k:.2f} sekund")
print(f"- Łączny czas treningu: {time_elapsed[-1]/60:.2f} minut")
print(f"- Łączna liczba kroków: {steps[-1]:,}".replace(",", " "))

# Znalezienie pierwszego indeksu, gdzie średnia nagroda przekracza wartość 2
threshold_index = np.argmax(mean_rewards > 6)

# Przycięcie danych do interesującego zakresu
steps_filtered = steps[threshold_index:]
mean_rewards_filtered = mean_rewards[threshold_index:]
std_rewards_filtered = std_rewards[threshold_index:]

# Dodatkowy wykres
plt.figure(figsize=(10, 5))
plt.plot(steps_filtered, mean_rewards_filtered, label='Średnia nagroda', color='blue', linewidth=2)
plt.axhline(y=2, color='purple', linestyle='--', label='Próg: 2')
plt.xlabel('Liczba kroków treningowych (od przekroczenia progu)', fontsize=12)
plt.ylabel('Wartość średniej nagrody', fontsize=12)
plt.title('Zmiany średniej nagrody od momentu przekroczenia wartości 6', fontsize=14, pad=20)
plt.legend(fontsize=10, loc='lower right')
plt.grid(True, linestyle='--', alpha=0.7)

plt.tight_layout()
plt.savefig("wykres2.png")
plt.show()

