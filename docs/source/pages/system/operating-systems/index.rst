Installing navi
---------------

Download the navi executable from `this link <https://github.com/denisidoro/navi/releases/download/v2.23.0/navi-v2.23.0-x86_64-unknown-linux-musl.tar.gz>`_

.. code-block::

  sudo apt install --yes fzf

Create a symlink to the OLAV navi cheatsheet to use it as the main cheatsheet when loading navi:

.. code-block::

  ln -s ~/Repositories/github.com/aarhus-robotics/olav/config/navi/olav.cheat  ~/.local/share/navi/cheats/main.cheat






chrony



Configure VNC access
--------------------

.. code-block::

  sudo apt install --yes x11vnc

First, create a password to access the VNC server 

.. code-block:: shell

    sudo x11vnc -storepasswd <your-password> ~/.vnc/passwd

Then, create a new `systemd` service to start the VNC server at system startup by editing a new service file under `/etc/systemd/system/x11vnc.service` with the following contents:

.. code-block:: shell

    [Unit]
    Description=Start x11vnc at startup.
    Requires=display-manager.service
    Requires=sddm.service
    After=display-manager.service
    After=sddm.service


    [Service]
    Type=simple
    ExecStartPre=/bin/bash -c "/bin/sleep 30 && /usr/bin/systemctl set-environment SDDMAUTH=$(find /var/run/sddm/ -type f)"
    ExecStart=/bin/bash -c "/usr/bin/x11vnc -display :0 -auth $SDDMAUTH -forever -loop -repeat -rfbauth /home/olav/.vnc/passwd -rfbport 5900 -noxdamage -noscr -noxfixes -nowf -xkb -ncache_cr -ncache 10"

    [Install]
    WantedBy=multi-user.target

Notice there is a hardcoded 30 seconds delay in the service to ensure SDDM successfully started and the display `:0` is available.

Enable the `systemd` service at startup - it is a good idea to check the service status immediately after to confirm the service started successfully:

.. code-block:: shell

    sudo systemctl enable --now x11vnc.service
    sudo systemctl status x11vnc.service


Install system utilities
========================


.. code-block:: shell

    sudo apt install --yes \
        bmon
        btop

Configure the terminal
^^^^^^^^^^^^^^^^^^^^^^

Add the following lines to `.zshrc`:

.. code-block:: shell

    # Add local binary directory to path
    path+=('/home/olav/.local/bin')
    export PATH
