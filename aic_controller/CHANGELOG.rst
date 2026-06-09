^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aic_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-06-09)
------------------
* Add top level LICENSE file (`#384 <https://github.com/intrinsic-dev/aic/issues/384>`_)
* Fix URDF header include (.h vs .hpp) to compile aic_controller in ROS 2 Jazzy (`#358 <https://github.com/intrinsic-dev/aic/issues/358>`_)
* Add `target_mode` field to `ControllerState` message (`#309 <https://github.com/intrinsic-dev/aic/issues/309>`_)
* change 'wrench_feedback_gains_at_tip' from Wrench to float64 (`#308 <https://github.com/intrinsic-dev/aic/issues/308>`_)
* Remove unused controllers (`#302 <https://github.com/intrinsic-dev/aic/issues/302>`_)
* Add service to tare force torque sensor within aic_controller (`#275 <https://github.com/intrinsic-dev/aic/issues/275>`_)
* Fix velocity target in different frames (`#289 <https://github.com/intrinsic-dev/aic/issues/289>`_)
* Add parameter for minimum error to trigger clamping (`#288 <https://github.com/intrinsic-dev/aic/issues/288>`_)
* reset controller target to current position if no progress is made towards target within a given timeout duration (`#277 <https://github.com/intrinsic-dev/aic/issues/277>`_)
* fix transform of reference velocity. It should be transformed into base_link frame (`#276 <https://github.com/intrinsic-dev/aic/issues/276>`_)
* reset feedforward wrenches upon deactivating controller so that any previously sent values does not carry over into the controller's reactivation (`#228 <https://github.com/intrinsic-dev/aic/issues/228>`_)
* Add offset_wrench parameter to offset the weight of payloads (`#199 <https://github.com/intrinsic-dev/aic/issues/199>`_)
* remove logging (`#179 <https://github.com/intrinsic-dev/aic/issues/179>`_)
* fix variable not assigned after clamping to min/max values (`#177 <https://github.com/intrinsic-dev/aic/issues/177>`_)
* Fix frame in which velocity targets are applied and tune impedance parameters (`#109 <https://github.com/intrinsic-dev/aic/issues/109>`_)
* Implement the Joint Impedance Action (`#91 <https://github.com/intrinsic-dev/aic/issues/91>`_)
* Implement the cartesian impedance controller (`#67 <https://github.com/intrinsic-dev/aic/issues/67>`_)
* Implement cartesian limit clamping and linear interpolation (`#57 <https://github.com/intrinsic-dev/aic/issues/57>`_)
* Add style check workflow and lint code (`#55 <https://github.com/intrinsic-dev/aic/issues/55>`_)
* Minor cosmetic change to command_interface_configuration() (`#54 <https://github.com/intrinsic-dev/aic/issues/54>`_)
* AIC Controller Skeleton (`#41 <https://github.com/intrinsic-dev/aic/issues/41>`_)
* Contributors: John TGZ, John Tan, Luca Della Vedova, Morgan Quigley, Teo Koon Peng, Trushant Adeshara, Yadunund, yadunund
