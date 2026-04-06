Building the Vizard Unity Project
=================================

These instructions allow you to compile and run Vizard within the Unity application.

#. **Install the Unity Hub.** If it is not yet installed, visit `Unity <https://unity.com>`__
   and follow the platform-specific instructions to install the Unity Hub.

#. **Install the Unity 6000.0 LTS editor.** Vizard is currently based on Unity
   ``6000.0.71f1``.

   a. In Unity Hub, click ``Installs`` on the left side.
   b. Click ``Install Editor`` in the upper-right corner.
   c. Select the recommended Unity ``6000.0.xx`` LTS release. It should be
      ``6000.0.71f1`` or newer.
   d. Install any desired optional components, such as:

      * Visual Studio Code
      * Linux Build Support (Mono)
      * macOS Build Support (IL2CPP)
      * Windows Build Support (Mono)

#. **Open the Vizard Unity project.**

   a. Start Unity Hub.
   b. Click the ``Add`` dropdown in the upper-right corner and select
      ``Add Project From Disk``.
   c. Navigate to ``Vizard/VizardUnityProject`` and click ``Open``.
   d. In the Unity Hub project list, click the newly added project to open it.

   .. note::

      If your installed Unity 6000.0 version does not match the version last used by the
      repository, Unity will ask you to confirm opening the project in a non-matching
      editor. Click ``Yes`` to continue.

#. **Load the startup scene.** After ``VizardUnityProject`` finishes importing, type
   ``VizardStartupScene`` into the Project search bar and double-click the scene asset
   to open it.
#. **Install the TMP Essentials Unity package.** When Unity displays the ``TMP Importer``
   panel, click ``Import TMP Essentials``. The ``TMP Examples & Extras`` package is
   optional and can be skipped.
#. **Test the local installation.** Press the Unity ``Play`` button and, in the Game
   window, use ``Select`` to open a Basilisk scenario ``.bin`` file and confirm the
   setup works.
#. **Optional: Install a C# IDE.** A C# IDE is recommended for script editing. Both
   Visual Studio and JetBrains Rider provide optional packages for Unity development.

.. important::

   The atmosphere shader materials for Earth, Mars, and Venus, together with
   installation instructions, are available through the ``Vizard_HD_Materials``
   bundle listed on :ref:`vizardDownload`.
   The atmosphere and ring shader used in Vizard are adapted from the
   `Planet Shader and Shadowing System <https://assetstore.unity.com/packages/vfx/shaders/planet-shader-and-shadowing-system-49693>`__
   package available in the Unity Asset Store.
