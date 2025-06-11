import re
import matplotlib.pyplot as plt
import numpy as np

"""
Mean Reward - the average value of rewards received by the model at a given training stage.
An increase in this value indicates that the model is learning to make better decisions. Ideally, we should observe a gradual increase in this value.

Std of Reward - measures the variability of the rewards received. A high standard deviation indicates that the model sometimes achieves very good and sometimes very poor results.
A gradual decrease in this value is desirable as the model's performance stabilizes.
"""

steps = []
mean_rewards = []
std_rewards = []
time_elapsed = []

pattern = re.compile(
    r'Step: (\d+)\. Time Elapsed: ([\d.]+) s\. Mean Reward: ([\d.-]+)\. Std of Reward: ([\d.-]+)\.'
)

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
plt.plot(steps, mean_rewards, label='Mean Reward', color='blue', linewidth=2)
plt.axhline(y=median_mean_reward, color='red', linestyle='--',
            label=f'Median: {median_mean_reward:.2f}')
plt.axhline(y=average_mean_reward, color='green', linestyle='--',
            label=f'Average: {average_mean_reward:.2f}')
plt.xlabel('Training Steps', fontsize=12)
plt.ylabel('Mean Reward Value', fontsize=12)
plt.title('Changes in Mean Reward During Training', fontsize=14, pad=20)
plt.legend(fontsize=10, loc='lower right')
plt.grid(True, linestyle='--', alpha=0.7)

plt.subplot(1, 2, 2)
plt.plot(steps, std_rewards, label='Standard Deviation', color='orange', linewidth=2)
plt.xlabel('Training Steps', fontsize=12)
plt.ylabel('Standard Deviation Value', fontsize=12)
plt.title('Changes in Reward Standard Deviation', fontsize=14, pad=20)
plt.legend(fontsize=10, loc='upper right')
plt.grid(True, linestyle='--', alpha=0.7)

plt.tight_layout()
plt.savefig("plot1.png")
plt.show()

print("\nTRAINING STATISTICS SUMMARY:")
print(f"- Average reward value throughout training: {average_mean_reward:.3f}")
print(f"- Median reward value: {median_mean_reward:.3f}")
print(f"- Average time per 10,000 steps: {average_time_per_10k:.2f} seconds")
print(f"- Total training time: {time_elapsed[-1]/60:.2f} minutes")
print(f"- Total number of steps: {steps[-1]:,}".replace(",", " "))

# Find the first index where the mean reward exceeds 6
threshold_index = np.argmax(mean_rewards > 6)

# Filter data from the point where the threshold is exceeded
steps_filtered = steps[threshold_index:]
mean_rewards_filtered = mean_rewards[threshold_index:]
std_rewards_filtered = std_rewards[threshold_index:]

# Additional plot
plt.figure(figsize=(10, 5))
plt.plot(steps_filtered, mean_rewards_filtered, label='Mean Reward', color='blue', linewidth=2)
plt.axhline(y=6, color='purple', linestyle='--', label='Threshold: 6')
plt.xlabel('Training Steps (from threshold exceedance)', fontsize=12)
plt.ylabel('Mean Reward Value', fontsize=12)
plt.title('Changes in Mean Reward After Exceeding Value 6', fontsize=14, pad=20)
plt.legend(fontsize=10, loc='lower right')
plt.grid(True, linestyle='--', alpha=0.7)

plt.tight_layout()
plt.savefig("plot2.png")
plt.show()