Download Vizard Source Code
===========================

.. important::

   Vizard is a Unity game engine project and requires the `Unity Editor <https://unity.com/>`__
   to open the project, run it locally, and build platform-specific binaries.

.. image:: /_images/static/basiliskVizardLogo.png
   :align: right
   :scale: 50 %

The Vizard source code is hosted openly on `GitHub <https://github.com>`__. Visit
the `AVSLab Vizard repository <https://github.com/AVSLab/vizard>`__ and download or
clone the source code.

Vizard is developed using the Git version control system. The following steps
explain how to clone the repository or pull updates to an existing local copy:

#. If needed, create your own `GitHub <https://github.com>`__ account.
#. Open the `Vizard GitHub repository <https://github.com/AVSLab/vizard>`__ in a browser.
#. In the repository clone panel, select the ``https`` option instead of ``ssh``.
#. Copy the project URL ``https://github.com/AVSLab/vizard.git``.
#. Clone the repository with your preferred Git client. If using SourceTree, select
   ``Clone From URL``, paste the Vizard repository URL, and select the ``develop``
   branch to pull the latest code.
#. Enable Git LFS in your local repository and pull the large-file content. Vizard
   uses Git LFS to store texture and model files.

   a. Install `Git LFS <https://git-lfs.com/>`__ if needed.
   b. In a terminal, navigate to your local repository and enable Git LFS::

         git lfs install

   c. Pull the Vizard Git LFS content::

         git lfs pull

The local Vizard project is then ready to be opened with the Unity Editor and compiled.
