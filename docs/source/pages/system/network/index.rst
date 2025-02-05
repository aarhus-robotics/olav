Network
=======

OLAV provides its own local area network (LAN) to allow reliable communication between all subsystems and to provide the
user direct access to controls and metrics.

Devices
-------

All devices are assigned a static IP by a DHCP server under the same subnet. The DHCP server is provided by a Teltonika
RUT955 router router. The DHCP reserves IP addresses for each device based on the MAC address of the device. Below is a list of the DHCP leases:

.. csv-table:: Network topology
   :file: network-topology.csv

Mobile network
--------------

OLAV is configured with a single SIM card for mobile network access in Denmark and abroad. The mobile network is used to provide continuous VPN access for remote debugging and to receive RTK corrections from an NTRIP server during navigation.

The SIM card number is `+45 93 52 16 67`.

Virtual private network
-----------------------

Seleted devices in the OLAV network can be accessed remotely using through a virtual private network. The virtual
private network allows for SSH access, as well as a direct hook into the ROS network OLAV is running.

Be mindful that the network has limited throughput, and large ROS messages such as those representing point cloud or
image data may not be delivered in a timely manner or may not be delivered at all through the VPN.

To connect to the server, you will need OpenVPN installed. On Ubuntu machines, this can be installed using the APT
package manager:

.. code-block:: shell

   sudo apt install --yes openvpn

Retrieve the certificate generated for your device. If you do not have a valid certificate, please send an access
request to `Dario Sirangelo <mailto:dsi@aarhusrobotics.com>`_ to receive a certificate and a matching static IP address
in the VPN.

Copy the OpenVPN configuration file (with file extension `.conf`) under `/etc/openvpn/client`. If you would like to run
automatically connect to the OLAV VPN on startup, you can enable a `systemd` service at startup with the following
command:

.. code-block:: shell

   sudo systemctl enable openvpn-client@<your-client-name>

Below is a list of the devices accessible through the VPN and their respective static IP addresses:

.. csv-table:: VPN topology
   :file: vpn-topology.csv
