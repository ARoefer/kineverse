import kineverse.gradients.gradient_math as gm


def cart_force_to_q_forces(cartesian_point, cartesian_force, symbols):
    """Generates a column vector of configuration space forces
       that exert a desired Cartesian force.
    
    Args:
        cartesian_point  (point3): Point at which the force shall be exerted
        cartesian_force (vector3): Force vector to exert
        symbols (iterable): Symbols that shall be considered in the force 
                            calculation. Order determines the order of torque 
                            vector.
    Return:
        (Matrix) Torque column vector.
    """
    return gm.dot(gm.jacobian(cartesian_point, symbols).T, cartesian_force)
