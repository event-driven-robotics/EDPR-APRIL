#!/bin/bash
echo "Creating aliases for edpr-april Docker"
echo "\n\n# ===== EDPR-APRIL docker aliases =====" >> ~/.bashrc
echo "alias start-test='xhost + && docker start event-pose && docker exec -it event-pose bash && clear'" >> ~/.bashrc
echo "alias test='docker exec -it event-pose bash && clear'" >> ~/.bashrc
echo "alias stop-tes='docker stop event-pose && clear'" >> ~/.bashrc