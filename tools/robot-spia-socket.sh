#!/bin/sh
# Gira SUL ROBOT (busybox sh), lanciato da tools/spia-robot.sh socket / socket-ferma.
# Inserisce "spia socket" tra AuxCtrl e RobotApp senza fermare AuxCtrl (e quindi il GD32):
#   1. spia sul socket stream /tmp/AuxEverest_Sock (server = AuxCtrl);
#   2. termina everest-server (il client di quel socket): Monitor lo rilancia e si collega attraverso la spia;
#   3. solo dopo, spia sul socket a datagrammi /tmp/UDP_AUXCTRL_SEND_PATH, che crea RobotApp stesso
#      (RobotApp non viene riavviato, quindi il suo socket resta quello intercettato).
D=/tmp/rtes

avvia() {
	pidof AuxCtrl >/dev/null && pidof RobotApp >/dev/null || { echo "lo stack originale non gira"; exit 1; }
	ora=$(date +%Y%m%d-%H%M%S)
	[ -e $D/socket.reg ] && mv $D/socket.reg $D/socket-$ora.reg
	[ -e $D/socket-udp.reg ] && mv $D/socket-udp.reg $D/socket-udp-$ora.reg

	start-stop-daemon -S -b -m -p $D/sock-stream.pid -x /bin/sh -- -c \
		"exec $D/spia socket -o $D/socket.reg -s 10 --stream aux,everest,/tmp/AuxEverest_Sock >$D/sock-stream.out 2>$D/sock-stream.err"
	sleep 1
	cat $D/sock-stream.out
	[ -S /tmp/AuxEverest_Sock.vero ] || { echo "ERRORE: la spia non si è inserita"; cat $D/sock-stream.err; exit 1; }

	vecchio=$(pidof everest-server)
	kill $vecchio
	echo "everest-server $vecchio terminato, aspetto che Monitor lo rilanci e che si colleghi..."
	i=0
	while [ $i -lt 60 ]; do
		n=$(pidof everest-server)
		if [ -n "$n" ] && [ "$n" != "$vecchio" ] && grep -q "nuovo client" $D/sock-stream.err; then break; fi
		sleep 1
		i=$((i + 1))
	done
	echo "everest-server nuovo: $(pidof everest-server) (dopo $i s)"
	grep "nuovo client" $D/sock-stream.err || echo "ATTENZIONE: everest-server non si è collegato alla spia"

	sleep 1
	start-stop-daemon -S -b -m -p $D/sock-udp.pid -x /bin/sh -- -c \
		"exec $D/spia socket -o $D/socket-udp.reg -s 10 --dgram udp_auxctrl,grezzo,/tmp/UDP_AUXCTRL_SEND_PATH >$D/sock-udp.out 2>$D/sock-udp.err"
	sleep 1
	cat $D/sock-udp.out $D/sock-udp.err
}

ferma() {
	kill $(cat $D/sock-udp.pid 2>/dev/null) 2>/dev/null
	kill $(cat $D/sock-stream.pid 2>/dev/null) 2>/dev/null
	sleep 1
	ls /tmp/*.vero 2>/dev/null && echo "ATTENZIONE: socket .vero rimasti"
	tail -n 2 $D/sock-stream.err
	# everest-server era collegato attraverso la spia: si riavvia tutto lo stack per tornare puliti
	/etc/init.d/robotManager stop
	kill $(pidof Monitor RobotApp AuxCtrl log-server everest-server) 2>/dev/null
	sleep 2
	kill -9 $(pidof Monitor RobotApp AuxCtrl log-server everest-server) 2>/dev/null
	/etc/init.d/robotManager start
	sleep 3
	echo "stack: Monitor=$(pidof Monitor) AuxCtrl=$(pidof AuxCtrl) RobotApp=$(pidof RobotApp)"
}

stato() {
	echo "spia stream=$(cat $D/sock-stream.pid 2>/dev/null) udp=$(cat $D/sock-udp.pid 2>/dev/null) AuxCtrl=$(pidof AuxCtrl) RobotApp=$(pidof RobotApp)"
	tail -n 2 $D/sock-stream.err
	tail -n 1 $D/sock-udp.err
}

case "$1" in
	avvia) avvia ;;
	ferma) ferma ;;
	stato) stato ;;
	*) echo "uso: $0 avvia|ferma|stato"; exit 2 ;;
esac
