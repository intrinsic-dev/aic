^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aic_model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.1.0 (2026-06-09)
------------------
* Update lerobot to fix pixi environment conflicts (`#467 <https://github.com/intrinsic-dev/aic/issues/467>`_) (`#492 <https://github.com/intrinsic-dev/aic/issues/492>`_)
* Add top level LICENSE file (`#384 <https://github.com/intrinsic-dev/aic/issues/384>`_)
* move_robot callback (`#306 <https://github.com/intrinsic-dev/aic/issues/306>`_)
* Make set_target_mode calls sync to avoid later deadlocks (`#355 <https://github.com/intrinsic-dev/aic/issues/355>`_)
* Shutdown model and validate behavior (`#350 <https://github.com/intrinsic-dev/aic/issues/350>`_)
* Use sim time in all policies (`#327 <https://github.com/intrinsic-dev/aic/issues/327>`_)
* Add `target_mode` field to `ControllerState` message (`#309 <https://github.com/intrinsic-dev/aic/issues/309>`_)
* change 'wrench_feedback_gains_at_tip' from Wrench to float64 (`#308 <https://github.com/intrinsic-dev/aic/issues/308>`_)
* Reduce time limit to 3 min and document in rules (`#305 <https://github.com/intrinsic-dev/aic/issues/305>`_)
* Add tf2_ros dependency to package.xml and update pixi.toml for tf2-ros-py (`#312 <https://github.com/intrinsic-dev/aic/issues/312>`_)
* move policy instantiation into on_configure (`#244 <https://github.com/intrinsic-dev/aic/issues/244>`_)
* Rename policy class to just 'Policy' (`#241 <https://github.com/intrinsic-dev/aic/issues/241>`_)
* fix type hints (`#207 <https://github.com/intrinsic-dev/aic/issues/207>`_)
* Imperative (polled) API for policies (`#172 <https://github.com/intrinsic-dev/aic/issues/172>`_)
* Add logging of joint and pose commands to scoring (`#168 <https://github.com/intrinsic-dev/aic/issues/168>`_)
* Add abstract methods to PolicyRos (`#153 <https://github.com/intrinsic-dev/aic/issues/153>`_)
* update set_pose_target to accept reference frame (`#158 <https://github.com/intrinsic-dev/aic/issues/158>`_)
* Trigger observation cb only if goal_handle is active (`#151 <https://github.com/intrinsic-dev/aic/issues/151>`_)
* Move goal_completed logic to the example policy (`#146 <https://github.com/intrinsic-dev/aic/issues/146>`_)
* Factor policy out of `aic_model_node` (`#116 <https://github.com/intrinsic-dev/aic/issues/116>`_)
* Remove `aic_model/files.txt` (`#114 <https://github.com/intrinsic-dev/aic/issues/114>`_)
* Add keyboard end effector teleop (`#103 <https://github.com/intrinsic-dev/aic/issues/103>`_)
* aic_model pose command publisher for a minimal demo (`#102 <https://github.com/intrinsic-dev/aic/issues/102>`_)
* Add aic_model image, zenoh router access control and docker-compose (`#89 <https://github.com/intrinsic-dev/aic/issues/89>`_)
* Align Observation TCP transform with aic_controller frame (`#101 <https://github.com/intrinsic-dev/aic/issues/101>`_)
* Contributors: Carlos Aguero, Carlos Agüero, Ivan David Martinez Baron, John TGZ, John Tan, Luca Della Vedova, Morgan Quigley, Teo Koon Peng, Trushant Adeshara, Yadunund
