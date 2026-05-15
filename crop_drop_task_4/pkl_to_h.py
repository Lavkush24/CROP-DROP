# import pickle

# with open("q_table3.pkl", "rb") as f:
#     q_table = pickle.load(f)

# with open("q_table.h", "w") as f:
#     f.write("#ifndef Q_TABLE_H\n#define Q_TABLE_H\n\n")
#     f.write("#define NUM_STATES 125\n#define NUM_ACTIONS 5\n\n")
#     f.write("float q_table[NUM_STATES][NUM_ACTIONS] = {\n")
#     for s in range(125):
#         q_vals = q_table.get(float(s), [0.0] * 5)
#         row = ", ".join(f"{v:.4f}f" for v in q_vals)
#         f.write(f"    {{ {row} }},\n")
#     f.write("};\n\n#endif // Q_TABLE_H\n")

import pickle
import numpy as np

# Load the Q-table from the pickle file
with open("/home/venkatesh/Documents/cb/RL trainning/q_table10.pkl", "rb") as f:
    data = pickle.load(f)

q_table = data['q_table']
num_states, num_actions = q_table.shape

# Generate the C header file
with open("q_table10.h", "w") as h_file:
    h_file.write("#ifndef Q_TABLE_H\n")
    h_file.write("#define Q_TABLE_H\n\n")
    h_file.write(f"#define NUM_STATES {num_states}\n")
    h_file.write(f"#define NUM_ACTIONS {num_actions}\n\n")
    h_file.write("float q_table[NUM_STATES][NUM_ACTIONS] = {\n")

    for i, row in enumerate(q_table):
        row_str = ", ".join(f"{val:.6f}" for val in row)
        end_char = "," if i < num_states - 1 else ""
        h_file.write(f"    {{ {row_str} }}{end_char}\n")

    h_file.write("};\n\n")
    h_file.write("#endif // Q_TABLE_H\n")
