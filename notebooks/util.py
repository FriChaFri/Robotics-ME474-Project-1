import numpy as np
import sympy as sp
def print_latex_expression(expr, rounding=None, delimiter=None):
    """
    Converts a NumPy array, SymPy Matrix, or SymPy expression into a LaTeX formatted string,
    with optional rounding and delimiter wrapping, and prints it to the terminal.
    
    Parameters:
        expr (numpy.ndarray, sympy.Matrix, or sympy.Basic): The input array or expression.
        rounding (int or None): Number of decimal places for rounding numeric values.
                                If a string (i.e. '$' or '$$') is provided here and no delimiter
                                is specified, it is interpreted as the delimiter.
        delimiter (str or None): If provided (should be '$' or '$$'), the output LaTeX string
                                 is wrapped with the delimiter on both sides (with a space).
    
    Returns:
        None (prints the LaTeX formatted string to the terminal)
    """
    # If the second parameter is a string and no delimiter is provided,
    # treat it as the delimiter and set rounding to None.
    if isinstance(rounding, str) and delimiter is None:
        delimiter = rounding
        rounding = None

    # Validate the delimiter if provided.
    if delimiter is not None and delimiter not in {'$', '$$'}:
        raise ValueError("Delimiter must be either '$' or '$$'.")

    def round_if_numeric(element, rounding):
        # Attempt to round if the element has no free symbols.
        try:
            if element.free_symbols == set():
                return sp.Float(round(float(element), rounding))
        except Exception:
            pass
        return element

    # Process a NumPy array.
    if isinstance(expr, np.ndarray):
        if rounding is not None:
            expr = np.around(expr, decimals=rounding)
        expr = sp.Matrix(expr)
    
    # Process a SymPy Matrix.
    elif isinstance(expr, sp.Matrix):
        if rounding is not None:
            expr = sp.Matrix(expr.rows, expr.cols,
                             lambda i, j: round_if_numeric(expr[i, j], rounding))
    
    # Process a SymPy expression or symbol.
    elif isinstance(expr, sp.Basic):
        if rounding is not None:
            try:
                if expr.free_symbols == set():
                    expr = sp.Float(round(float(expr), rounding))
            except Exception:
                pass
    else:
        raise TypeError("Input must be a NumPy ndarray, a SymPy Matrix, or a SymPy expression.")

    # Generate the LaTeX string.
    latex_str = sp.latex(expr)

    # Wrap with delimiter if provided (with a space after and before the delimiters).
    if delimiter is not None:
        latex_str = f"{delimiter} {latex_str} {delimiter}"

    # Print the resulting LaTeX string to the terminal.
    print("$$" + latex_str + "$$") 