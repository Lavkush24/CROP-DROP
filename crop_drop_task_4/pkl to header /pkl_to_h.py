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

with open("q_table.pkl", "rb") as f:
    data = pickle.load(f)

q_table = data["q_table"]
num_states, num_actions = q_table.shape

with open("q_table.h", "w") as h:
    h.write("#ifndef Q_TABLE1_H\n")
    h.write("#define Q_TABLE1_H\n\n")

    h.write(f"#define NUM_STATES {num_states}\n")
    h.write(f"#define NUM_ACTIONS {num_actions}\n\n")

    h.write("const float q_table[NUM_STATES][NUM_ACTIONS] = {\n")

    for i, row in enumerate(q_table):
        row_str = ", ".join(f"{v:.6f}f" for v in row)
        comma = "," if i < num_states - 1 else ""
        h.write(f"    {{ {row_str} }}{comma}\n")

    h.write("};\n\n")
    h.write("#endif // Q_TABLE1_H\n")
