^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fuse_loss
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2023-03-03)
------------------

1.0.0 (2023-03-03)
------------------
* fuse -> ROS 2 : Doc Generation (`#278 <https://github.com/locusrobotics/fuse/issues/278>`_)
  * Port doc generation and fix package.xml for linting
  * Fix small bugs in package.xml
  * Use default rosdoc2 settings
  * Use default rosdoc2 settings
  * Update fuse_doc for rosdoc2
  ---------
  Co-authored-by: Shane Loretz <sloretz@google.com>
* fuse -> ROS 2 fuse_models: Port fuse_models (`#304 <https://github.com/locusrobotics/fuse/issues/304>`_)
  * Port messages
  * Port fuse_models
  * Fix alloc error and some bugs
  * Wait on result
* fuse -> ROS 2 fuse_graphs : Port fuse_graphs (`#289 <https://github.com/locusrobotics/fuse/issues/289>`_)
* fuse -> ROS 2 fuse_variables: Linting (`#296 <https://github.com/locusrobotics/fuse/issues/296>`_)
  * Migrate to .hpp files
  * Create redirection headers
  * Make xmllint and uncrustify happy
  * Wrap most comment lines
  * Satisfy cpplint
* fuse -> ROS 2 fuse_loss: Linting (`#295 <https://github.com/locusrobotics/fuse/issues/295>`_)
* fuse -> ROS 2 fuse_loss: Port fuse_loss (`#287 <https://github.com/locusrobotics/fuse/issues/287>`_)
* fuse -> ROS 2 fuse_core: Linting (`#292 <https://github.com/locusrobotics/fuse/issues/292>`_)
* fuse -> ROS 2: Clean up macro usage warnings (`#280 <https://github.com/locusrobotics/fuse/issues/280>`_)
* fuse -> ROS 2 fuse_msgs : Port package and ignore unported packages for now (`#277 <https://github.com/locusrobotics/fuse/issues/277>`_)
  Co-authored-by: Tom Moore <tmoore@locusrobotics.com>
* Adding doxygen to all packages (`#241 <https://github.com/locusrobotics/fuse/issues/241>`_)
* Contributors: Tom Moore, methylDragon

0.4.2 (2021-07-20)
------------------
* Adding roslint dependency to fuse_viz (`#231 <https://github.com/locusrobotics/fuse/issues/231>`_)
  * Adding roslint dependency to fuse_viz
  * Silence CMP0048 warnings
* Contributors: Tom Moore

0.4.1 (2021-07-13)
------------------
* Changelogs
* Fix roslint 0.12.0 (`#186 <https://github.com/locusrobotics/fuse/issues/186>`_)
  * Fix roslint 0.12.0 include_what_you_use warnings
  Mostly for:
  * std::move -> #include <utility>
  * std::make_shared and similar -> #include <memory>
  * Remove static string variable not permitted by roslint 0.12.0, using a test fixture where needed.
* Replace ignition_sensors list param with ignition field (`#163 <https://github.com/locusrobotics/fuse/issues/163>`_)
  * Remove ignition_sensors param and use a per-sensor ignition field
* Add ComposedLoss (`#170 <https://github.com/locusrobotics/fuse/issues/170>`_)
* Patch Tukey loss for Ceres < 2.0.0 (`#159 <https://github.com/locusrobotics/fuse/issues/159>`_)
  * Patch Tukey loss for Ceres < 2.0.0
  * Create ceres_macros.h header
* Plot loss (`#143 <https://github.com/locusrobotics/fuse/issues/143>`_)
  * Add test to plot loss rho, influence and weight
  * Add BUILD_WITH_PLOT_TESTS option (defaults OFF)
* Remove Pseudo-Huber loss, it duplicates SoftLOne (`#152 <https://github.com/locusrobotics/fuse/issues/152>`_)
  * Remove Pseudo-Huber loss, it duplicates SoftLOne
* Add new loss functions (`#142 <https://github.com/locusrobotics/fuse/issues/142>`_)
  The following loss functions, not available in Ceres solver are
  provided:
  * Geman-McClure
  * DCS (Dynamic Covariance Scaling)
  * Fair
  * Pseudo-Huber
  * Welsch
* Support ScaledLoss (`#141 <https://github.com/locusrobotics/fuse/issues/141>`_)
* Add fuse_loss pkg with plugin-based loss functions (`#118 <https://github.com/locusrobotics/fuse/issues/118>`_)
* Contributors: Enrique Fernandez Perdomo, Stephen Williams, Tom Moore

* Fix roslint 0.12.0 (`#186 <https://github.com/locusrobotics/fuse/issues/186>`_)
  * Fix roslint 0.12.0 include_what_you_use warnings
  Mostly for:
  * std::move -> #include <utility>
  * std::make_shared and similar -> #include <memory>
  * Remove static string variable not permitted by roslint 0.12.0, using a test fixture where needed.
* Replace ignition_sensors list param with ignition field (`#163 <https://github.com/locusrobotics/fuse/issues/163>`_)
  * Remove ignition_sensors param and use a per-sensor ignition field
* Add ComposedLoss (`#170 <https://github.com/locusrobotics/fuse/issues/170>`_)
* Patch Tukey loss for Ceres < 2.0.0 (`#159 <https://github.com/locusrobotics/fuse/issues/159>`_)
  * Patch Tukey loss for Ceres < 2.0.0
  * Create ceres_macros.h header
* Plot loss (`#143 <https://github.com/locusrobotics/fuse/issues/143>`_)
  * Add test to plot loss rho, influence and weight
  * Add BUILD_WITH_PLOT_TESTS option (defaults OFF)
* Remove Pseudo-Huber loss, it duplicates SoftLOne (`#152 <https://github.com/locusrobotics/fuse/issues/152>`_)
  * Remove Pseudo-Huber loss, it duplicates SoftLOne
* Add new loss functions (`#142 <https://github.com/locusrobotics/fuse/issues/142>`_)
  The following loss functions, not available in Ceres solver are
  provided:
  * Geman-McClure
  * DCS (Dynamic Covariance Scaling)
  * Fair
  * Pseudo-Huber
  * Welsch
* Support ScaledLoss (`#141 <https://github.com/locusrobotics/fuse/issues/141>`_)
* Add fuse_loss pkg with plugin-based loss functions (`#118 <https://github.com/locusrobotics/fuse/issues/118>`_)
* Contributors: Enrique Fernandez Perdomo, Stephen Williams

0.4.0 (2019-07-12)
------------------

0.3.0 (2019-03-18)
------------------

0.2.0 (2019-01-16)
------------------

0.1.1 (2018-08-15)
------------------

0.1.0 (2018-08-12)
------------------

0.0.2 (2018-07-16)
------------------

0.0.1 (2018-07-05)
------------------
