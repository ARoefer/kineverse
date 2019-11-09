The Math
========

Kineverse uses symbolic expressions to encode parameterized mathematical relation ships. Most of the logic behind this system is provided by the awesome SymenginePy library. However, Kineverse adds some of its own twists to that system which are .. important to know about.

Types and Derivatives
---------------------

Since Kineverse is focused on compatibility with gradient based optimization methods, it is important to pay special attention to the concept of derivatives. 

The (partial) derivative :math:`\frac{\Delta f}{\Delta x}` of a function :math:`f` is a mapping between the rate of change of variable :math:`x` and the rate of change of function :math:`f`. If :math:`x` refers to a position, then the rate of change is a velocity, if :math:`x` refers to a velocity, then its rate of change would be an acceleration. To interpret a gradient meaningfully it is important to understand to know the "type" of :math:`x`. By default, the Symengine library does not draw this distinction, which is why Kineverse provides its own tools for doing so.

The logic for types is defined in :math:`gradients.diff_logic`.

