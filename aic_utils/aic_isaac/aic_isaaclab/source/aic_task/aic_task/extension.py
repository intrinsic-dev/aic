# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import omni.ext
import os

EXTENSION_PATH = os.path.dirname(os.path.abspath(__file__))
# source/aic_task/aic_task/tasks/manager_based/aic_task/Intrinsic_assets/assets
ASSETS_PATH = os.path.join(
    EXTENSION_PATH, "tasks/manager_based/aic_task/Intrinsic_assets", "assets"
)


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class ExampleExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[aic_task] startup")

        self._window = omni.ui.Window("AIC Debug", width=300, height=300)
        with self._window.frame:
            with omni.ui.VStack():
                # label = omni.ui.Label("AIC Debug")
                with omni.ui.VStack():
                    omni.ui.Button(
                        "Import Plugs", height=50, clicked_fn=self.import_plugs
                    )
                    omni.ui.Button(
                        "Create Cable", height=50, clicked_fn=self.create_cable
                    )
                    omni.ui.Button(
                        "Reset Orient", height=50, clicked_fn=self.reset_orient_op_type
                    )

    def on_shutdown(self):
        print("[aic_task] shutdown")

    def create_cable(self):
        print("[aic_task] create_cable")

        from pxr import Usd, UsdGeom, Gf, UsdPhysics, UsdShade, Sdf, PhysxSchema
        from omni.physx.scripts import physicsUtils
        import omni.usd

        stage = omni.usd.get_context().get_stage()

        # configure ropes (all units in meters):
        linkRadius = 0.003  # capsule radius
        linkHalfLength = (
            0.02  # capsule cylindrical height; increase to use fewer, longer links
        )
        capsuleHeight = 1.5 * linkHalfLength + linkRadius
        ropeLength = 0.5  # total rope length
        ropeColor = Gf.Vec3f(0.1, 0.5, 0.1)
        coneAngleLimit = 80
        rope_damping = 0.1
        rope_stiffness = 1.0
        contactOffset = linkRadius * 0.02  # small relative to link size

        # stage = get_current_stage()
        prim_path = "/World/Rope"
        # Define Xform prim for the rope
        rope_prim = UsdGeom.Xform.Define(stage, prim_path)
        # add translate op to the rope prim
        rope_prim.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
        # add rotate op xyz to the rope prim
        rope_prim.AddRotateXYZOp().Set(Gf.Vec3d(0, 0, 0))
        # add scale op to the rope prim
        rope_prim.AddScaleOp().Set(Gf.Vec3d(1, 1, 1))

        # Define PhysicsMaterial prim for the rope
        UsdShade.Material.Define(stage, f"{prim_path}/PhysicsMaterial")
        material = UsdPhysics.MaterialAPI.Apply(
            stage.GetPrimAtPath(f"{prim_path}/PhysicsMaterial")
        )
        material.CreateStaticFrictionAttr().Set(0.1)
        material.CreateDynamicFrictionAttr().Set(0.1)
        material.CreateRestitutionAttr().Set(0)

        linkLength = 2.0 * linkHalfLength - linkRadius
        numLinks = max(1, int(ropeLength / linkLength))
        xStart = 2 * linkLength - 0.5 * capsuleHeight  # -numLinks * linkLength * 0.5
        yStart = 0.0

        scopePath = Sdf.Path(prim_path).AppendChild("Rope")
        UsdGeom.Scope.Define(stage, scopePath)

        y = yStart
        z = 0.0
        jointX = linkHalfLength - 0.5 * linkRadius

        # Create one capsule prim per link
        linkPaths = []
        for linkInd in range(numLinks):
            linkPath = scopePath.AppendChild(f"link_{linkInd}")
            linkPaths.append(linkPath)

            capsuleGeom = UsdGeom.Capsule.Define(stage, linkPath)
            capsuleGeom.CreateHeightAttr(capsuleHeight)
            capsuleGeom.CreateRadiusAttr(linkRadius)
            capsuleGeom.CreateAxisAttr("X")
            capsuleGeom.CreateDisplayColorAttr().Set([ropeColor])

            x = xStart + linkInd * linkLength
            UsdGeom.Xformable(capsuleGeom).AddTranslateOp().Set(Gf.Vec3d(x, y, z))

            prim = capsuleGeom.GetPrim()
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.RigidBodyAPI.Apply(prim)
            massAPI = UsdPhysics.MassAPI.Apply(prim)
            massAPI.CreateDensityAttr().Set(0.0005)
            physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            physxCollisionAPI.CreateRestOffsetAttr().Set(0.0)
            physxCollisionAPI.CreateContactOffsetAttr().Set(contactOffset)
            physicsUtils.add_physics_material_to_prim(
                stage, prim, f"{prim_path}/PhysicsMaterial"
            )

        # Create one joint prim per consecutive link pair
        for linkInd in range(numLinks - 1):
            jointPath = scopePath.AppendChild(f"joint_{linkInd}_{linkInd + 1}")
            joint = UsdPhysics.Joint.Define(stage, jointPath)

            joint.GetBody0Rel().SetTargets([linkPaths[linkInd]])
            joint.GetBody1Rel().SetTargets([linkPaths[linkInd + 1]])
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(jointX, 0, 0))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(-jointX, 0, 0))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

            d6Prim = joint.GetPrim()

            # Locked DOFs (low > high means locked)
            for dof in ["transX", "transY", "transZ", "rotX"]:
                limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, dof)
                limitAPI.CreateLowAttr(1.0)
                limitAPI.CreateHighAttr(-1.0)

            # Free DOFs with drives for rope dynamics
            for dof in ["rotY", "rotZ"]:
                limitAPI = UsdPhysics.LimitAPI.Apply(d6Prim, dof)
                limitAPI.CreateLowAttr(-coneAngleLimit)
                limitAPI.CreateHighAttr(coneAngleLimit)

                driveAPI = UsdPhysics.DriveAPI.Apply(d6Prim, dof)
                driveAPI.CreateTypeAttr("force")
                driveAPI.CreateDampingAttr(rope_damping)
                driveAPI.CreateStiffnessAttr(rope_stiffness)

        # Define a USD PhysicsFixedJoint
        fixedJoint = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/fixedJoint")
        fixedJoint.GetBody0Rel().SetTargets([linkPaths[0]])
        fixedJoint.GetBody1Rel().SetTargets([Sdf.Path("/World/sc_plug_visual")])

        sc_plug_prim = stage.GetPrimAtPath("/World/sc_plug_visual")
        massAPI = UsdPhysics.MassAPI.Apply(sc_plug_prim)
        massAPI.CreateDensityAttr().Set(0.0005)
        physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(sc_plug_prim)
        physxCollisionAPI.CreateRestOffsetAttr().Set(0.0)
        physxCollisionAPI.CreateContactOffsetAttr().Set(contactOffset)
        physicsUtils.add_physics_material_to_prim(
            stage, sc_plug_prim, f"{prim_path}/PhysicsMaterial"
        )

        # make sc_plug_prim an articulation root
        # UsdPhysics.ArticulationRootAPI.Apply(sc_plug_prim)

        # Define a USD PhysicsFixedJoint
        fixedJoint2 = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/fixedJoint2")
        fixedJoint2.GetBody0Rel().SetTargets([linkPaths[-1]])
        fixedJoint2.GetBody1Rel().SetTargets([Sdf.Path("/World/lc_plug_visual")])

        lc_plug_prim = stage.GetPrimAtPath("/World/lc_plug_visual")
        massAPI = UsdPhysics.MassAPI.Apply(lc_plug_prim)
        massAPI.CreateDensityAttr().Set(0.0005)
        physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(lc_plug_prim)
        physxCollisionAPI.CreateRestOffsetAttr().Set(0.0)
        physxCollisionAPI.CreateContactOffsetAttr().Set(contactOffset)
        physicsUtils.add_physics_material_to_prim(
            stage, lc_plug_prim, f"{prim_path}/PhysicsMaterial"
        )

    def import_plugs(self):
        from pxr import Gf, UsdGeom, UsdPhysics, Sdf

        print(f"Extension Path: {EXTENSION_PATH}")

        print(f"Assets Path: {ASSETS_PATH}")
        # list assets
        print(os.listdir(ASSETS_PATH))

        # add sc_plug_visual.usd to the stage
        sc_plug_file_path = os.path.join(ASSETS_PATH, "SC Plug", "sc_plug_visual.usd")
        # Add reference to the stage
        stage = omni.usd.get_context().get_stage()
        sc_plug_prim = stage.DefinePrim("/World/sc_plug_visual", "Xform")
        sc_plug_prim.GetReferences().AddReference(sc_plug_file_path)
        sc_plug_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(0, 0, 0))
        sc_plug_prim.GetAttribute("xformOp:orient").Set(Gf.Quatf(0, 0, 0, -1.0))
        sc_plug_prim.GetAttribute("xformOp:scale").Set(Gf.Vec3d(0.01, 0.01, 0.01))
        sc_plug_prim.GetAttribute("xformOpOrder").Set(
            ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
        )

        # add lc plug: LC Plug/lc_plug_assemble.usd
        ropeLength = 0.55
        lc_plug_file_path = os.path.join(ASSETS_PATH, "LC Plug", "lc_plug_assembly.usd")
        # Add reference to the stage
        lc_plug_prim = stage.DefinePrim("/World/lc_plug_visual", "Xform")
        lc_plug_prim.GetReferences().AddReference(lc_plug_file_path)
        lc_plug_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(ropeLength, 0, 0))
        lc_plug_prim.GetAttribute("xformOp:orient").Set(Gf.Quatf(0.7071, 0, 0, -0.7071))
        lc_plug_prim.GetAttribute("xformOp:scale").Set(Gf.Vec3d(0.01, 0.01, 0.01))
        lc_plug_prim.GetAttribute("xformOpOrder").Set(
            ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
        )

        # # add stp_module: SFP Module/sfp_module_visual.usd
        # stp_module_file_path = os.path.join(ASSETS_PATH, "SFP Module", "sfp_module_visual.usd")
        # # Add reference to the stage
        # stp_module_prim = stage.DefinePrim("/World/stp_module_visual", "Xform")
        # stp_module_prim.GetReferences().AddReference(stp_module_file_path)
        # stp_module_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(ropeLength + 0.03, 0, 0))
        # stp_module_prim.GetAttribute("xformOp:orient").Set(Gf.Quatf(0.5, 0.5, -0.5, -0.5))
        # stp_module_prim.GetAttribute("xformOp:scale").Set(Gf.Vec3d(0.01, 0.01, 0.01))
        # stp_module_prim.GetAttribute("xformOpOrder").Set(["xformOp:translate", "xformOp:orient", "xformOp:scale"])

    def reset_orient_op_type(self):
        import omni.usd
        from pxr import UsdGeom, Gf, Sdf

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(
            "/World/ur5e/cable_0218/lc_plug_visual"
        )  # <-- change to your prim path

        # Step 1: Read the current quatf value
        orient_attr = prim.GetAttribute("xformOp:orient")
        quatf_val = orient_attr.Get()
        print(f"Before: {quatf_val}, type: {orient_attr.GetTypeName()}")  # quatf

        # Step 2: Convert Gf.Quatf → Gf.Quatd
        quatd_val = Gf.Quatd(
            float(quatf_val.GetReal()), Gf.Vec3d(*quatf_val.GetImaginary())
        )

        # Step 3: Remove the old quatf attribute from the edit target layer
        edit_layer = stage.GetEditTarget().GetLayer()
        prim_spec = edit_layer.GetPrimAtPath(prim.GetPath())

        if prim_spec and "xformOp:orient" in prim_spec.properties:
            prim_spec.RemoveProperty(prim_spec.properties["xformOp:orient"])

        # Step 4: Recreate the attribute as quatd and set the value
        new_attr = prim.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatd)
        new_attr.Set(quatd_val)

        print(f"After:  {new_attr.Get()}, type: {new_attr.GetTypeName()}")  # quatd
