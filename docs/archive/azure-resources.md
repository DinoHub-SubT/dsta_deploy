# Azure & Terraform Resource References

## VPN Connection (Reference)

VPN connections is costly to keep up. So, please destroy the VPN when not using VMs and re-create when ready to connect to the VMs.

### Create a VPN Connection

        subtf_mkvpn.sh

- will take approximately 25 - 35 minutes.

To see more options:

        subtf_mkvpn.sh --help

### Destroy an existing VPN Connection

        subtf_rmvpn.sh

- will take approximately 15 minutes.

To see more options:

        subtf_rmvpn.sh --help


### Manual Setup

**Please use `subtf_mkvpn.sh` and `subtf_rmvpn.sh`. The below is just a reference for manual VPN setup.**

The below instructions can be found on [azure tutorials](https://docs.microsoft.com/en-us/azure/vpn-gateway/point-to-site-vpn-client-configuration-azure-cert) for reference.

  - Please follow the below instructions and use the *azure tutorials* only when running into issues or for more background information.

**Install Dependency Libraries**

      sudo apt-get update
      sudo apt-get install strongswan strongswan-pki libstrongswan-extra-plugins network-manager-strongswan

**Create the Root & User Certificates**

      # create the cert path
      mkdir -p ~/.ssh/azure/vpn
      cd ~/.ssh/azure/vpn

      # Create the Root CA cert
      ipsec pki --gen --outform pem > caKey.pem
      ipsec pki --self --in caKey.pem --dn "CN=VPN CA" --ca --outform pem > caCert.pem

      # Copy the output to the 'vpn_ca_cert' variable to '~/.terraform_flags.bashrc'
      openssl x509 -in caCert.pem -outform der | base64 -w0 ; echo

      # == Create the user certificate ==

      # Please change 'password' to something more secure
      export PASSWORD="password"
      # Please change 'username' to your username
      export USERNAME="client"

      # generate the user certificate
      ipsec pki --gen --outform pem > "${USERNAME}Key.pem"
      ipsec pki --pub --in "${USERNAME}Key.pem" | ipsec pki --issue --cacert caCert.pem --cakey caKey.pem --dn "CN=${USERNAME}" --san "${USERNAME}" --flag clientAuth --outform pem > "${USERNAME}Cert.pem"

      # generate the p12 bunder
      openssl pkcs12 -in "${USERNAME}Cert.pem" -inkey "${USERNAME}Key.pem" -certfile caCert.pem -export -out "${USERNAME}.p12" -password "pass:${PASSWORD}"

**Update personalized terraform variables**

Change the personalized cert key variable in the `~/.terraform_flags.bashrc` terraform:

        gedit ~/.terraform_flags.bashrc

        # change `vpn_ca_cert` to the output seen in the terminal

**Apply Changes to Azure**

      # apply the VPN gateway, this can take up to 30 minutes, just for the VPN. It can be longer if setting up more resources
      cd ~/deploy_ws/src/operations/azurebooks/subt/

      # Dry-run: shows the user the azure deployment
      terraform plan

      # Apply the terraform setup to azure
      terraform apply

**Download the VPN Client**

      # go to a ssh folder to contain your vpn keys
      cd ~/.ssh/azure/vpn

      # Get the vpn client, this will output a https link, please remember it!
      #   - the 'vnet gateway name' is: [resource_name_prefix]-vnet-gateway
      #   - where the 'resource_name_prefix' was set in the '~/.terraform_flags.bashrc' in the previous steps
      #   - an example would be: USERNAME-example-vnet-gateway
      az network vnet-gateway vpn-client generate --name [vnet gateway name] --processor-architecture x86 --resource-group SubT

      # download the client (without brackets)
      # if the wget command does not work, put the https link (from the previous step) in your browser and download it to '~/.ssh/azure/vpn' location
      wget --directory-prefix=. [https path from previous command WITHOUT QUOTES ]

      # unzip the vpn client package
      #   - its okay to ignore the warnings '1 archive had warnings but no fatal errors.'
      unzip -j -d client-download [downloaded.zip]

      # == Connect to the VPN ==

      # Get the VPN server DNS, copy the name between '<VpnServer>' tags
      cd client-download
      grep -rni VpnServer

**Setup Networking GUI Plugin**

To setup the GUI, please follow the [instructions here](https://docs.microsoft.com/en-us/azure/vpn-gateway/point-to-site-vpn-client-configuration-azure-cert#install).

Summary of above link (please use the link):

- Open the Network Manager Ubuntu GUI

- Add a new `VPN` connection, make sure it is the `IPsec/IKEv2 (strongswan)` connection

- Add the `[client]Cert.pem`, `[client]Key.pem` and the VPN server DNS name between `<VpnServer>` tags (from the previous step).

- Select `Request an inner IP address`

- Select the VPN connection

- Select the folder icon at the end of the Certificate field, browse to the Generic folder, and select the VpnServerRoot file.

## Remote Desktop

The subt terraform example has remote desktop port enabled.
  - The user needs to install `rdp` package dependencies on the VM and on the localhost client

**Connect to your VM using VPN**

        # ssh into your VM
        ssh [username]@[private IP]

- **To find the IP:**

    - Go to the Azure Portal Website

    - Or, run the command below, with `resource_name_prefix` as set previously in `~/.terraform_flags.bashrc`:

            az vm list-ip-addresses -g SubT -o table | grep [resource_name_prefix]

**Install desktop enviornment (remote host VM)**

        sudo apt-get update
        sudo apt-get -y install xfce4
        sudo apt-get -y install xrdp
        sudo systemctl enable xrdp
        echo xfce4-session >~/.xsession
        sudo service xrdp restart

**Create the RDP client script (localhost)**

        # install rdp client dependency
        sudo apt-get install rdesktop

        # edit the params here:
        vim operations/scripts/example-remote-desktop.bash

        # close the script & make it executable
        chmod +x my-rdp-client.bash

        # connect to rdp server
        ./my-rdp-client.bash

- `rdp` requires a password. There is no password setup for the default user.

    - Continue with the build tutorials where the `ansible` scripts will set a default password in the VMs for you.
    - Or, set the `subt` user's password manually in the VM.

- `rdp` services needs to be restarted on VM reboot:

    - Continue with the build tutorials where the `ansible` scripts will setup the `xrdp` service to start on VM boot for you.

    - Or restart the service manually in the VM:

            sudo systemctl enable xrdp
            echo xfce4-session >~/.xsession
            sudo service xrdp restart

* * *
