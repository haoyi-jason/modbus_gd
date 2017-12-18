# List files for freemodbus

FMBSRC = ${CHIBIOS}/ext/freemodbus_gd/mb_hy.c \
		 ${CHIBIOS}/ext/freemodbus_gd/ascii/mbascii.c \
		 ${CHIBIOS}/ext/freemodbus_gd/rtu/mbcrc.c \
		 ${CHIBIOS}/ext/freemodbus_gd/rtu/mbrtu.c \
		 ${CHIBIOS}/ext/freemodbus_gd/tcp/mbtcp.c \
		 ${CHIBIOS}/ext/freemodbus_gd/functions/mbfunccoils.c \
		 ${CHIBIOS}/ext/freemodbus_gd/functions/mbfuncdiag.c \
		 ${CHIBIOS}/ext/freemodbus_gd/functions/mbfuncdisc.c \
		 ${CHIBIOS}/ext/freemodbus_gd/functions/mbfuncholding.c \
		 ${CHIBIOS}/ext/freemodbus_gd/functions/mbfuncinput.c \
		 ${CHIBIOS}/ext/freemodbus_gd/functions/mbfuncother.c \
		 ${CHIBIOS}/ext/freemodbus_gd/functions/mbutils.c \
		 
		 
FMBINC =${CHIBIOS}/ext/freemodbus_gd \
		${CHIBIOS}/ext/freemodbus_gd/ascii \
		${CHIBIOS}/ext/freemodbus_gd/include \
		${CHIBIOS}/ext/freemodbus_gd/rtu \
		${CHIBIOS}/ext/freemodbus_gd/tcp
