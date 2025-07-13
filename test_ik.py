#!/usr/bin/env python3
"""
Test script to verify the SCARA inverse kinematics implementation
"""
import math

def test_ik_standalone():
    """Test the IK equations with the provided example values"""
    
    # SCARA parameters
    a1 = l1 = 110  # first link length
    a2 = l2 = 110  # second link length
    d = 25.5      # distance between arm origins

    # Test position
    x = 1
    y = 7
    
    print("SCARA Inverse Kinematics Test")
    print("=" * 40)
    print(f"Parameters: l1={l1}, l2={l2}, d={d}")
    print(f"Target position: ({x}, {y})")
    print()
    
    # Calculate distances
    c = math.sqrt((x**2) + (y**2))
    e = math.sqrt(((d-x)**2) + (y**2))
    
    print(f"Distance calculations:")
    print(f"c = sqrt(x² + y²) = sqrt({x}² + {y}²) = {c:.4f}")
    print(f"e = sqrt((d-x)² + y²) = sqrt(({d}-{x})² + {y}²) = {e:.4f}")
    print()
    
    # Original calculation from your code
    t1 = math.atan(y/x) + math.acos(((a1**2) + (c**2) - (a2**2))/(2*a1*c))
    t2 = (math.atan(y/(d-x)) + math.acos(((a1**2) + (e**2) - (a2**2))/(2*a1*e)))
    
    print("Original calculation:")
    print(f"t1 = {math.degrees(t1):.2f}°")
    print(f"t2 = {180 - math.degrees(t2):.2f}°")
    print()
    
    # Step-by-step calculation matching the diagram
    print("Step-by-step calculation:")
    
    # Left arm (α)
    alpha1 = math.atan2(y, x)
    print(f"α1 = atan2({y}, {x}) = {math.degrees(alpha1):.4f}°")
    
    cos_arg_left = ((l1**2) + (c**2) - (l2**2)) / (2 * l1 * c)
    print(f"cos argument for α2 = ({l1}² + {c:.4f}² - {l2}²) / (2 × {l1} × {c:.4f}) = {cos_arg_left:.4f}")
    
    alpha2 = math.acos(cos_arg_left)
    print(f"α2 = acos({cos_arg_left:.4f}) = {math.degrees(alpha2):.4f}°")
    
    alpha = alpha1 + alpha2
    print(f"α = α1 + α2 = {math.degrees(alpha1):.4f}° + {math.degrees(alpha2):.4f}° = {math.degrees(alpha):.2f}°")
    print()
    
    # Right arm (β)
    beta1 = math.atan2(y, (d - x))
    print(f"β1 = atan2({y}, {d}-{x}) = atan2({y}, {d-x}) = {math.degrees(beta1):.4f}°")
    
    cos_arg_right = ((l1**2) + (e**2) - (l2**2)) / (2 * l1 * e)
    print(f"cos argument for β2 = ({l1}² + {e:.4f}² - {l2}²) / (2 × {l1} × {e:.4f}) = {cos_arg_right:.4f}")
    
    beta2 = math.acos(cos_arg_right)
    print(f"β2 = acos({cos_arg_right:.4f}) = {math.degrees(beta2):.4f}°")
    
    beta = math.pi - (beta1 + beta2)
    print(f"β = 180° - (β1 + β2) = 180° - ({math.degrees(beta1):.4f}° + {math.degrees(beta2):.4f}°) = {math.degrees(beta):.2f}°")
    print()
    
    print("Final Results:")
    print(f"Left arm angle (α):  {math.degrees(alpha):.2f}°")
    print(f"Right arm angle (β): {math.degrees(beta):.2f}°")
    print()
    print("Comparison with original:")
    print(f"Original t1: {math.degrees(t1):.2f}°, New α: {math.degrees(alpha):.2f}° (diff: {abs(math.degrees(t1) - math.degrees(alpha)):.2f}°)")
    print(f"Original t2: {180 - math.degrees(t2):.2f}°, New β: {math.degrees(beta):.2f}° (diff: {abs((180 - math.degrees(t2)) - math.degrees(beta)):.2f}°)")

if __name__ == "__main__":
    test_ik_standalone()
