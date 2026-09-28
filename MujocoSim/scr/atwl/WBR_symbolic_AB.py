from pathlib import Path
import re

import numpy as np
import sympy as sp


PARAMETER_FILE = Path(__file__).with_name("lqr物理参数.txt")
TABLE_ROW_RE = re.compile(
    r"^\s*"
    r"([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*,\s*"
    r"([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*,\s*"
    r"([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)"
    r"\s*$"
)


def load_parameter_table(filename):
    rows = []
    for line_number, line in enumerate(filename.read_text(encoding="utf-8").splitlines(), 1):
        match = TABLE_ROW_RE.match(line)
        if match is not None:
            rows.append(tuple(float(value) for value in match.groups()))

    if len(rows) != 7:
        raise ValueError(
            f"{filename} must contain exactly 7 table rows, found {len(rows)}"
        )

    lengths = np.asarray([row[0] for row in rows], dtype=float)
    if not np.all(np.diff(lengths) > 0.0):
        raise ValueError("Table leg lengths must be strictly increasing")
    if not np.all(np.isfinite(rows)):
        raise ValueError("Table values must all be finite")
    return rows


# ==================== Physical parameters ====================
Rw = 0.058
Rl = 0.403 / 2

parameter_table = load_parameter_table(PARAMETER_FILE)
leg_lengths = np.asarray([row[0] for row in parameter_table], dtype=float)
leg_inertias = np.asarray([row[1] for row in parameter_table], dtype=float)
wheel_com_distances = np.asarray([row[2] for row in parameter_table], dtype=float)

# The values below are the default single-point parameters used by the original
# script.  They are kept as a quick reference and as the default output check.
default_table_index = int(np.flatnonzero(np.isclose(leg_lengths, 0.25))[0])
ll = leg_lengths[default_table_index]
lr = leg_lengths[default_table_index]
lwl = wheel_com_distances[default_table_index]
lwr = wheel_com_distances[default_table_index]
Ill = leg_inertias[default_table_index]
Ilr = leg_inertias[default_table_index]

lc = -0.04177296

mw = 0.47
ml = 0.88403
mb = 10.51

Iw = 0.00039991

Ib = 0.14966292
Iz = 0.36308 * 2

g = 9.81


def build_linearized_model():
    """Build the equilibrium linearization with six table parameters symbolic."""
    ll_sym, lr_sym, lwl_sym, lwr_sym, Ill_sym, Ilr_sym = sp.symbols(
        "ll lr lwl lwr Ill Ilr"
    )

    # ==================== Symbolic variables ====================
    s, ds, dds = sp.symbols("s ds dds")
    phi, dphi, ddphi = sp.symbols("phi dphi ddphi")

    thll, dthll, ddthll = sp.symbols("thll dthll ddthll")
    thlr, dthlr, ddthlr = sp.symbols("thlr dthlr ddthlr")
    thb, dthb, ddthb = sp.symbols("thb dthb ddthb")

    Twl, Twr, Tbl, Tbr = sp.symbols("Twl Twr Tbl Tbr")

    fl, fr = sp.symbols("fl fr")

    Fwsl, Fwsr = sp.symbols("Fwsl Fwsr")
    Fwhl, Fwhr = sp.symbols("Fwhl Fwhr")
    Fbsl, Fbsr = sp.symbols("Fbsl Fbsr")
    Fbhl, Fbhr = sp.symbols("Fbhl Fbhr")

    # ==================== Generalized coordinates ====================
    q = sp.Matrix([s, phi, thll, thlr, thb])
    dq = sp.Matrix([ds, dphi, dthll, dthlr, dthb])
    ddq = sp.Matrix([dds, ddphi, ddthll, ddthlr, ddthb])

    u = sp.Matrix([Twl, Twr, Tbl, Tbr])

    # ==================== Kinematics ====================
    thwl = (s - Rl * phi - ll_sym * sp.sin(thll)) / Rw
    thwr = (s + Rl * phi - lr_sym * sp.sin(thlr)) / Rw

    sb = s + ll_sym / 2 * sp.sin(thll) + lr_sym / 2 * sp.sin(thlr)
    hb = ll_sym / 2 * sp.cos(thll) + lr_sym / 2 * sp.cos(thlr)

    lbl = ll_sym - lwl_sym
    lbr = lr_sym - lwr_sym
    sll = s - Rl * phi - lbl * sp.sin(thll)
    slr = s + Rl * phi - lbr * sp.sin(thlr)

    hll = hb - lbl * sp.cos(thll)
    hlr = hb - lbr * sp.cos(thlr)

    # ==================== Total derivative ====================
    def Dq(function):
        return sp.Matrix([sp.diff(function, qi) for qi in q]).T * dq

    def Dt(function):
        term1 = sum(
            sp.diff(function, qi) * dqi for qi, dqi in zip(q, dq)
        )
        term2 = sum(
            sp.diff(function, dqi) * ddqi for dqi, ddqi in zip(dq, ddq)
        )
        return sp.simplify(term1 + term2)

    dthwl = Dq(thwl)[0]
    dthwr = Dq(thwr)[0]

    ddthwl = Dt(dthwl)
    ddthwr = Dt(dthwr)

    dsb = Dq(sb)[0]
    dhb = Dq(hb)[0]

    ddsb = Dt(dsb)
    ddhb = Dt(dhb)

    dsll = Dq(sll)[0]
    dslr = Dq(slr)[0]

    dhll = Dq(hll)[0]
    dhlr = Dq(hlr)[0]

    ddsll = Dt(dsll)
    ddslr = Dt(dslr)

    ddhll = Dt(dhll)
    ddhlr = Dt(dhlr)

    # ==================== Dynamics ====================
    eq1 = mw * Rw * ddthwl - fl + Fwsl
    eq2 = mw * Rw * ddthwr - fr + Fwsr

    eq3 = Iw * ddthwl - Twl + fl * Rw
    eq4 = Iw * ddthwr - Twr + fr * Rw

    eq5 = ml * ddsll - Fwsl + Fbsl
    eq6 = ml * ddslr - Fwsr + Fbsr

    eq7 = ml * ddhll - Fwhl + Fbhl + ml * g
    eq8 = ml * ddhlr - Fwhr + Fbhr + ml * g

    eq9 = (
        Ill_sym * ddthll
        - (Fwhl * lwl_sym + Fbhl * lbl) * sp.sin(thll)
        + (Fwsl * lwl_sym + Fbsl * lbl) * sp.cos(thll)
        + Twl
        - Tbl
    )

    eq10 = (
        Ilr_sym * ddthlr
        - (Fwhr * lwr_sym + Fbhr * lbr) * sp.sin(thlr)
        + (Fwsr * lwr_sym + Fbsr * lbr) * sp.cos(thlr)
        + Twr
        - Tbr
    )

    eq11 = mb * ddsb - Fbsl - Fbsr
    eq12 = mb * ddhb - Fbhl - Fbhr + mb * g

    eq13 = (
        Ib * ddthb
        + Tbl
        + Tbr
        + (Fbsl + Fbsr) * lc * sp.cos(thb)
        - (Fbhl + Fbhr) * lc * sp.sin(thb)
    )

    eq14 = Iz * ddphi + fl * Rl - fr * Rl
    eq15 = Fwhl - Fwhr

    eq = sp.Matrix(
        [
            eq1,
            eq2,
            eq3,
            eq4,
            eq5,
            eq6,
            eq7,
            eq8,
            eq9,
            eq10,
            eq11,
            eq12,
            eq13,
            eq14,
            eq15,
        ]
    )

    # ==================== Solve dynamics ====================
    z = sp.Matrix(
        [
            dds,
            ddphi,
            ddthll,
            ddthlr,
            ddthb,
            fl,
            fr,
            Fwsl,
            Fwsr,
            Fwhl,
            Fwhr,
            Fbsl,
            Fbsr,
            Fbhl,
            Fbhr,
        ]
    )

    x = sp.Matrix(
        [
            s,
            ds,
            phi,
            dphi,
            thll,
            dthll,
            thlr,
            dthlr,
            thb,
            dthb,
        ]
    )

    # equationsToMatrix equivalent: Aeq * z = beq.
    eq_vars = list(z)
    Aeq, beq = sp.linear_eq_to_matrix(list(eq), eq_vars)

    # First solve the equilibrium accelerations and constraint forces, then
    # differentiate the implicit equations at that equilibrium.
    op_subs = {xi: 0 for xi in x}
    op_subs.update({ui: 0 for ui in u})

    Aeq0 = Aeq.subs(op_subs)
    beq0 = beq.subs(op_subs)
    z0 = Aeq0.LUsolve(beq0)

    op_subs.update({zi: z0[i] for i, zi in enumerate(z)})

    Jz0 = eq.jacobian(z).subs(op_subs)
    Jx0 = eq.jacobian(x).subs(op_subs)
    Ju0 = eq.jacobian(u).subs(op_subs)

    dzdx0 = -Jz0.LUsolve(Jx0)
    dzdu0 = -Jz0.LUsolve(Ju0)

    A_value = sp.zeros(10, 10)
    B_value = sp.zeros(10, 4)

    # f = [ds, dds, dphi, ddphi, dthll, ddthll, dthlr, ddthlr, dthb, ddthb]
    A_value[0, 1] = 1
    A_value[2, 3] = 1
    A_value[4, 5] = 1
    A_value[6, 7] = 1
    A_value[8, 9] = 1

    for row, z_row in zip([1, 3, 5, 7, 9], range(5)):
        A_value[row, :] = dzdx0[z_row, :]
        B_value[row, :] = dzdu0[z_row, :]

    return (A_value, B_value), (
        ll_sym,
        lr_sym,
        lwl_sym,
        lwr_sym,
        Ill_sym,
        Ilr_sym,
    )


def make_ab_evaluator():
    (a_expression, b_expression), parameter_symbols = build_linearized_model()
    return sp.lambdify(
        parameter_symbols,
        (a_expression, b_expression),
        modules="numpy",
        cse=True,
    )


def calculate_ab(evaluator, values):
    a_value, b_value = evaluator(*values)
    a_value = np.asarray(a_value, dtype=float)
    b_value = np.asarray(b_value, dtype=float)
    if a_value.shape != (10, 10) or b_value.shape != (10, 4):
        raise ValueError(f"Unexpected matrix dimensions: A={a_value.shape}, B={b_value.shape}")
    if not np.all(np.isfinite(a_value)) or not np.all(np.isfinite(b_value)):
        raise ValueError("A/B contains a non-finite value")
    return a_value, b_value


def parameter_combinations():
    for left_index in range(len(parameter_table)):
        for right_index in range(len(parameter_table)):
            yield (
                left_index,
                right_index,
                (
                    leg_lengths[left_index],
                    leg_lengths[right_index],
                    wheel_com_distances[left_index],
                    wheel_com_distances[right_index],
                    leg_inertias[left_index],
                    leg_inertias[right_index],
                ),
            )


def print_matrix(name, matrix):
    print(f"{name} =")
    print(
        np.array2string(
            matrix,
            formatter={"float_kind": lambda value: f"{value:.8f}"},
        )
    )


def cpp_float(value):
    if abs(value) < 5.0e-15:
        return "0.0"
    return f"{value:.17g}"


def save_cpp_tables(filename, a_table, b_table):
    a_coordinates = (
        (1, 4),
        (1, 6),
        (3, 4),
        (3, 6),
        (5, 4),
        (5, 6),
        (7, 4),
        (7, 6),
        (9, 4),
        (9, 6),
        (9, 8),
    )
    b_coordinates = tuple(
        (row, column)
        for row in (1, 3, 5, 7, 9)
        for column in range(4)
    )

    with open(filename, "w", encoding="utf-8") as file:
        file.write("// Generated by WBR_symbolic_AB.py. Do not edit by hand.\n")
        file.write(
            "// A values are ordered as "
            + ", ".join(f"A[{row}][{column}]" for row, column in a_coordinates)
            + ".\n"
        )
        file.write(
            "const std::array<std::array<double, 11>, 49> kAtATable = {{\n"
        )
        for index, matrix in enumerate(a_table):
            left_index, right_index = divmod(index, 7)
            file.write(
                f"    // ll={leg_lengths[left_index]:.2f}, lr={leg_lengths[right_index]:.2f}\n"
            )
            file.write(
                "    {{"
                + ", ".join(cpp_float(matrix[row, column]) for row, column in a_coordinates)
                + "}},\n"
            )
        file.write("}};\n\n")

        file.write(
            "// B values are ordered as "
            + ", ".join(f"B[{row}][{column}]" for row, column in b_coordinates)
            + ".\n"
        )
        file.write(
            "const std::array<std::array<double, 20>, 49> kAtBTable = {{\n"
        )
        for index, matrix in enumerate(b_table):
            left_index, right_index = divmod(index, 7)
            file.write(
                f"    // ll={leg_lengths[left_index]:.2f}, lr={leg_lengths[right_index]:.2f}\n"
            )
            file.write(
                "    {{"
                + ", ".join(cpp_float(matrix[row, column]) for row, column in b_coordinates)
                + "}},\n"
            )
        file.write("}};\n")


if __name__ == "__main__":
    evaluator = make_ab_evaluator()
    a_table = []
    b_table = []

    for left_index, right_index, values in parameter_combinations():
        a_value, b_value = calculate_ab(evaluator, values)
        a_table.append(a_value)
        b_table.append(b_value)

    if len(a_table) != 49 or len(b_table) != 49:
        raise RuntimeError("Expected exactly 49 A/B matrix pairs")

    default_index = default_table_index * 7 + default_table_index
    print(
        "Generated "
        f"{len(a_table)} A/B pairs from {PARAMETER_FILE.name}; "
        f"default pair is index {default_index} "
        f"(ll={ll:.2f}, lr={lr:.2f})."
    )
    print_matrix("A_num", a_table[default_index])
    print_matrix("B_num", b_table[default_index])
    save_cpp_tables("A_B_mat.txt", a_table, b_table)
