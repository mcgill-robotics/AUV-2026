wget -O /tmp/adept.deb https://files.digilent.com/Software/Adept2%20Runtime/2.30.1/digilent.adept.runtime_2.30.1_arm64.deb
wget -O /tmp/waveforms.deb https://files.digilent.com/Software/Waveforms/3.24.4/digilent.waveforms_3.24.4_arm64.deb
sudo apt-get update 
sudo apt-get install -y /tmp/adept.deb /tmp/waveforms.deb && \