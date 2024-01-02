import matplotlib.pyplot as plt
import numpy as np

# Initial investment and maintenance cost
initial_investment = 631000  # in DKK
annual_maintenance = 176680  # 28% of initial investment for maintenance

# Original annual labor savings
original_annual_savings = 1321999  # in DKK

# Adjusted annual savings after subtracting maintenance cost
adjusted_annual_savings = original_annual_savings - annual_maintenance

# Calculate payback period with adjusted savings
payback_years = initial_investment / adjusted_annual_savings

# Years for plotting
years = np.arange(0, int(payback_years) + 2)

# Cumulative savings for each year with adjusted savings
cumulative_savings = np.array([adjusted_annual_savings * year for year in years])

# Plot
plt.figure(figsize=(10, 6))
plt.plot(years, cumulative_savings, marker='o', color='b', label='Cumulative Savings')
plt.axhline(y=initial_investment, color='r', linestyle='--', label='Initial Investment')

# Annotating the payback period
plt.annotate(f'Payback Period: {payback_years:.2f} years',
             xy=(payback_years, initial_investment), 
             xytext=(payback_years, initial_investment + 500000),
             arrowprops=dict(facecolor='black', arrowstyle='->'),
             fontsize=16)

plt.title('Payback Period Analysis (Including Maintenance Costs)', fontsize=16)
plt.xlabel('Years', fontsize=14)
plt.ylabel('Cumulative Savings (DKK)', fontsize=14)
plt.legend()
# Removed the grid line
plt.show()
