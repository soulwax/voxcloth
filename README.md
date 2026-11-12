<p align="center">
  <img src="assets/logo.svg" width="88" alt="voxcloth logo"/>
</p>

# voxcloth

Pure Dart Extended Position-Based Dynamics (XPBD) cloth & rope solver.

## Features

- **Unconditionally Stable XPBD Constraints (`XpbdWorld`)**: Sub-stepped Lagrange-multiplier solver with inverse-mass pinning, compliance scaling $\tilde{\alpha} = \alpha / \Delta t^2$, and exponential air drag.
- **Structural, Shear, Pin & Dihedral Bending Constraints**: Exact `DistanceConstraint`, `DihedralBendingConstraint`, and `PinConstraint`.
- **Aerodynamic Wind & Collider Contacts**: Surface-normal wind lift across cloth triangles plus `SphereConstraintCollider`, `CapsuleConstraintCollider`, and `PlaneConstraintCollider` with tangential Coulomb friction.
- **Builders (`ClothGridBuilder`, `RopeChainBuilder`)**: Instant procedural banners, capes, flags, and hanging chains ready for mesh vertex/normal updates.
