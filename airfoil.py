path = './xf-naca2412-il-200000.csv'
airfoilData = pd.read_csv(path, skiprows=10)
cl_alpha_data = airfoilData[['Alpha','Cl']]

from scipy.stats import linregress

# Plot the Cl-alpha data
# plt.plot(np.array(cl_alpha_data['Alpha']), np.array(cl_alpha_data['Cl']))

# Select the linear range of data points (adjust these indices)
linear_range_start = 25
linear_range_end = 85
linear_data = cl_alpha_data.iloc[linear_range_start:linear_range_end]

# Perform linear regression
slope, intercept, r_value, p_value, std_err = linregress(linear_data['Alpha'], linear_data['Cl'])

# Plot the linear regression line
# plt.plot(linear_data['Alpha'], slope * linear_data['Alpha'] + intercept, 'r')

# Print the calculated slope
# # print("Linear Slope (Cl-alpha):", slope)
# from scipy.stats import linregress

# # Plot the Cl-alpha data
# plt.plot(np.array(cl_alpha_data['Alpha']), np.array(cl_alpha_data['Cl']))

# # Select the linear range of data points (adjust these indices)
# linear_range_start = 25
# linear_range_end = 85
# linear_data = cl_alpha_data.iloc[linear_range_start:linear_range_end]

# # Perform linear regression
# slope, intercept, r_value, p_value, std_err = linregress(linear_data['Alpha'], linear_data['Cl'])

# # Plot the linear regression line
# plt.plot(linear_data['Alpha'], slope * linear_data['Alpha'] + intercept, 'r')

# # Print the calculated slope
# print("Linear Slope (Cl-alpha):", slope)

# # Show the plot
# plt.xlabel('Angle of Attack (Alpha)')
# plt.ylabel('Lift Coefficient (Cl)')
# plt.legend(['Cl-alpha data', 'Linear Regression'])
# plt.title('Cl-alpha Curve with Linear Regression')
# plt.grid(True)
# # plt.show()