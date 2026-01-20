## Workflow RPI

### Installation initiale (sur la RPI)
1. Cloner le repo :
   - `git clone <repo> /home/pi/mower-robot`
2. Backend :
   - `cd /home/pi/mower-robot/backend`
   - `python3 -m venv venv`
   - `./venv/bin/pip install -r requirements.txt`
3. Service systemd :
   - `sudo cp /home/pi/mower-robot/embedded/RPI/mower-backend.service /etc/systemd/system/`
   - `sudo systemctl daemon-reload`
   - `sudo systemctl enable mower-backend.service`
   - `sudo systemctl start mower-backend.service`

### Déploiement rapide depuis le PC
1. Exporter les variables :
   - `export RPI_HOST=192.168.1.50`
   - `export RPI_USER=pi`
2. Déployer :
   - `./embedded/RPI/deploy.sh`

### Frontend (sur le PC)
Utilise la WebSocket de la RPI :
- `VITE_WS_URL=ws://<IP_RPI>:8000/ws/robot`
(192.168.1.28)

### Protocole série Arduino
Le backend envoie :
- `L:<speed>,R:<speed>\n`
- `speed` dans `[-1.0, 1.0]`
