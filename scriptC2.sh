#!/bin/bash


#send the files

sudo scp ubuntu@10.0.0.5:/home/ubuntu/P3.txt /home/alexis/Dropbox/THESIS/Testbed_code/workspace/alexis/Results/5/
scp ubuntu@10.0.0.5:/home/ubuntu/P4.txt /home/alexis/Dropbox/THESIS/Testbed_code/workspace/alexis/Results/5/

scp ubuntu@10.0.0.5:/home/ubuntu/completionTimeGlobalC2.txt /home/alexis/Dropbox/THESIS/Testbed_code/workspace/alexis/Results/5/
scp ubuntu@10.0.0.5:/home/ubuntu/Pdpx-Pdip-Pe-Pm.txt /home/alexis/Dropbox/THESIS/Testbed_code/workspace/alexis/Results/5/

echo Done
