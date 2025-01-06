Computing
=========

Uploading to ERDA
-----------------

Rosbags recorded with OLAV can be automatically uploaded and stored on `ERDA <https://erda.au.dk>`_ for long-term
storage.

To upload your latest recorded Rosbags to ERDA, you can use the PyOLAV script `upload.py`:

.. code-block:: shell
    
    python3 upload.py

All bags listed under `/home/olav/ROS/bags` will be uploaded under the `aarhus-dynamics/olav` workspace in a folder
named 'Rosbags`.

The script relies on the deploy key being available under `~/.ssh/`.