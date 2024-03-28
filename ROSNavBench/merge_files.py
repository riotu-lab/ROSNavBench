import pandas as pd

# Replace these with your actual file names
file = ['/home/riotu/Downloads/csv_files_experiments/House_Experiment_1__.csv', '/home/riotu/Downloads/csv_files_experiments/House_Experiment_2__.csv', '/home/riotu/Downloads/csv_files_experiments/House_Experiment_3__.csv', '/home/riotu/Downloads/csv_files_experiments/_Bookstore_Experiment_1__.csv', '/home/riotu/Downloads/csv_files_experiments/_Bookstore_Experiment_2__.csv', '/home/riotu/Downloads/csv_files_experiments/_Bookstore_Experiment_03__.csv','/home/riotu/Downloads/csv_files_experiments/Narrow_turtlebot3_house_1_v3.csv']

# Initialize an empty list to store dataframes
dataframes = []

# Read each CSV file
df1 = pd.read_csv(file[0])
df1['environment'] = 'house'
df1['level'] = 'no_obstacles'
df2 = pd.read_csv(file[1])
df2['environment'] = 'house'
df2['level'] = 'static_obstacles'
df3 = pd.read_csv(file[2])
df3['environment'] = 'house'
df3['level'] = 'dynamic_obstacles'
df4 = pd.read_csv(file[3])
df4['environment'] = 'bookstore'
df4['level'] = 'no_obstacles'
df5 = pd.read_csv(file[4])
df5['environment'] = 'bookstore'
df5['level'] = 'static_obstacles'
df6 = pd.read_csv(file[5])
df6['environment'] = 'bookstore'
df6['level'] = 'dynamic_obstacles'
df7 = pd.read_csv(file[6])
df7['environment'] = 'house'
df7['level'] = 'narrow_path'
# Append the dataframe to the list
dataframes.append(df1)
dataframes.append(df2)
dataframes.append(df3)
dataframes.append(df4)
dataframes.append(df5)
dataframes.append(df6)
dataframes.append(df7)
# Concatenate all dataframes
concatenated_df = pd.concat(dataframes)

# Save the concatenated dataframe to a new CSV file
concatenated_df.to_csv('/home/riotu/Downloads/csv_files_experiments/combined_file_all_narrow___.csv', index=False)
