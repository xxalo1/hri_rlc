import numpy as np
import yaml, copy, pandas as pd

def Rx(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0],
                     [0, c,-s],
                     [0, s, c]], dtype=np.float32)

def load_inertial(yaml_or_dict):
    """Return {name: {'id','m','com','Ic'}} with numpy arrays."""
    data = yaml.safe_load(open(yaml_or_dict, "r"))
    model = {}
    for row in data["links"]:
        if row["name"] is "ee_with_vision":
            continue  # skip end-effector with vision sensor
        model[row["name"]] = {
            "id": int(row["id"]),
            "m": float(row["mass_kg"]),
            "com": np.array(row["com_m"], dtype=np.float32),
            "Ic":  np.array(row["Ic_kgm2"], dtype=np.float32),
        }
    return model

def rotated_inertials(model, R):
    """Return a NEW dict with c' = R c, Ic' = R Ic R^T (no in-place mutation)."""
    out = {}
    for name, v in model.items():
        out[name] = {
            "id": v["id"],
            "m":  v["m"],
            "com": R @ v["com"],
            "Ic":  R @ v["Ic"] @ R.T,
        }
    return out

def inertia_to_vec(I):
    """Flatten symmetric inertia to 6-tuple: [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]."""
    return np.array([I[0,0], I[1,1], I[2,2], I[0,1], I[0,2], I[1,2]], dtype=float)

def compare_models(before, after):
    rows = []
    for name in before.keys():
        cb, ca = before[name]["com"], after[name]["com"]
        Ib, Ia = before[name]["Ic"],  after[name]["Ic"]
        ib, ia = inertia_to_vec(Ib), inertia_to_vec(Ia)
        rows.append({
            "link": name,
            "com_x_b": cb[0], "com_x_a": ca[0], "d_com_x": ca[0]-cb[0],
            "com_y_b": cb[1], "com_y_a": ca[1], "d_com_y": ca[1]-cb[1],
            "com_z_b": cb[2], "com_z_a": ca[2], "d_com_z": ca[2]-cb[2],
            "Ixx_b": ib[0], "Ixx_a": ia[0], "d_Ixx": ia[0]-ib[0],
            "Iyy_b": ib[1], "Iyy_a": ia[1], "d_Iyy": ia[1]-ib[1],
            "Izz_b": ib[2], "Izz_a": ia[2], "d_Izz": ia[2]-ib[2],
            "Ixy_b": ib[3], "Ixy_a": ia[3], "d_Ixy": ia[3]-ib[3],
            "Ixz_b": ib[4], "Ixz_a": ia[4], "d_Ixz": ia[4]-ib[4],
            "Iyz_b": ib[5], "Iyz_a": ia[5], "d_Iyz": ia[5]-ib[5],
        })
    df = pd.DataFrame(rows).set_index("link")
    # Pretty print
    pd.set_option("display.width", 160)
    pd.set_option("display.max_columns", None)
    pd.options.display.float_format = "{:+.6e}".format
    print(df)
    return df

# --- run ---
model0 = load_inertial("kinova_gen3_inertial.yaml")  # your YAML file
before = copy.deepcopy(model0)                        # snapshot BEFORE rotation
Rflip  = Rx(np.pi)                                    # rotate frame by π about x
after  = rotated_inertials(before, Rflip)             # NEW dict, no mutation

print("\nCOM & inertia comparison (before vs after; deltas shown):")
_ = compare_models(before, after)
