--------------------------------------------------------------------------------
切换模式
--------------------------------------------------------------------------------
net mode <mode>
	- net mode sta							# 切换到STA模式
	- net mode ap							# 切换到softAP模式
	- net mode mon							# 切换到monitor模式


--------------------------------------------------------------------------------
STA模式
--------------------------------------------------------------------------------
net sta config <ssid> [psk]						# 配置ssid和psk，并对相关设置选项进行通用型配置，可连接常见配置的AP。
	- net sta config ssid_example				# 可连接open模式的AP。
	- net sta config ssid example				# 可连接open模式的AP。
	- net sta config ssid_example psk_example	# 可连接WPA/WPA2（TKIP/CCMP）模式的AP。

net sta set <field> <value>						# 单独对一种设置选项进行配置。
	- net sta set ssid ssid_example
	- net sta set psk psk_example

									# 配置wep_key，wep_key的有效形式（40bit或104bit）举例如下。
	- net sta set wep_key0 "abcde"				# 5位ASCII码（40bit）
	- net sta set wep_key1 "abcdefghijklm"		# 13位ASCII码（104bit）
	- net sta set wep_key2 0123456789			# 10位十六进制数（40bit）
	- net sta set wep_key3 0123456789abcdef0123456789		# 26位十六进制数（104bit）
	- net sta set wep_key_index (0, 1, 2, 3)

									# 以下几个｛｝中的选项可多选，以空格间隔。
	- net sta set key_mgmt {WPA-PSK, NONE}		# OPEN、WEP模式：NONE。WPA、WPA2、WPA/WPA2模式：WPA-PSK。
	- net sta set pairwise {CCMP, TKIP, WEP40, WEP104, NONE}
	- net sta set group {CCMP, TKIP, WEP40, WEP104, NONE}
	- net sta set proto {WPA, RSN}
	- net sta set auth_alg {OPEN, SHARED}		# OPEN、WPA、WPA2、WPA/WPA2模式：OPEN。WEP模式：SHARED。

	- net sta set ptk_rekey <seconds>
	- net sta set scan_ssid (0, 1)				# 设置为1时，扫描隐藏ssid的AP。

net sta get <field>								# 获取一种设置选项的配置值。
	- net sta get ssid
	- net sta get psk
	- net sta get wep_key0
	- net sta get wep_key1
	- net sta get wep_key2
	- net sta get wep_key3
	- net sta get wep_key_index
	- net sta get key_mgmt
	- net sta get pairwise
	- net sta get group
	- net sta get proto
	- net sta get auth_alg
	- net sta get ptk_rekey
	- net sta get scan_ssid

net sta enable
net sta disable

net sta scan once							# 扫描一次
net sta scan result <num>						# 获取最多<num>个AP的扫描结果
	- net sta scan result 20
net sta bss flush <age>							# 移除扫描结果中，未使用的并且超过<age>秒没更新的AP。
	- net sta bss flush 0
	- net sta bss flash 10

net sta connect
net sta disconnect

net sta wps pbc								# 开始PBC模式的WPS流程。
net sta wps pin								# 获取一个合法的PIN值。
net sta wps pin <pin>							# 设置一个PIN值，若PIN值合法，则开始PIN模式的WPS流程。
	- net sta wps pin set 01234567


--------------------------------------------------------------------------------
一次连接命令示例：
net sta set ssid MERCURY_AD96b


net sta set key_mgmt NONE
net sta set psk sw4wifionly


net sta config MERCURY_AD96b
net sta config TP-LINK_36DA6E
net sta config TES_HUAWEI_WS318#4 sw4wifionly
net sta config TES_TPLINK_WR941N#80 sw4wifionly
net sta config Tenda_5BE718#27 sw4wifionly
net sta config TES_MERCURY_AD96#008
net sta config TP-LINK_WR541G_#92 sw4wifionly
net sta config TES_MERCURY_1200R#45 sw4wifionly
net sta config TES_DLINK_DIR613#48 sw4wifionly
net sta config TP-LINK_D1CD
net sta config TES_TPLink_WR842N_#78 12345678
net sta config TES_FAST_FW450R#59 sw4wifionly
net sta config TES_TPLINK_WR886N#29 sw4wifionly
net sta config TES_NETGEAR_JNRD3000#23 sw4wifionly

net sta enable
net iperf tcp-recv 192.168.51.1 0

netcmd lmac parse_flags_set 0x7fff 0x7fff

--------------------------------------------------------------------------------
softAP模式
--------------------------------------------------------------------------------
net ap config <ssid> [psk]						# 配置ssid和psk，并对相关设置选项进行通用型配置，设置为常见配置的AP。
	- net ap config ssid_example					# 设置为OPEN模式的AP。
	- net ap config ssid_example psk_example			# 设置为WPA/WPA2（TKIP/CCMP）模式的AP。

net ap set <field> <value>						# 单独对一种设置选项进行配置。
	- net ap set ssid ssid_example
	- net ap set psk psk_example

									# 以下几个｛｝中的选项可多选，以空格间隔。
	- net ap set key_mgmt {WPA-PSK, NONE}				# OPEN模式：NONE。WPA、WPA2、WPA/WPA2模式：WPA-PSK。
	- net ap set wpa {CCMP, TKIP, NONE}
	- net ap set rsn {CCMP, TKIP, NONE}
	- net ap set proto <NONE, {WPA, RSN}>				# OPEN模式：NONE。WPA、WPA2、WPA/WPA2模式：{WPA, RSN}。
	- net ap set auth_alg {OPEN}

	- net ap set group_rekey <seconds>
	- net ap set strict_rekey <0, 1>				# 设置为1时，当有STA离开当前BSS时，更新GTK。
	- net ap set gmk_rekey <seconds>
	- net ap set ptk_rekey <seconds>
	- net ap set hw_mode <b, g>
	- net ap set 80211n <0, 1>
	- net ap set channel <1 ~ 13>
	- net ap set beacon_int <15 ~ 65535>
	- net ap set dtim <1 ~ 255>
	- net ap set max_num_sta <num>					# 设置同时连接AP的STA数量的上限

net ap get <field>									# 获取一种设置选项的配置值。
	- net ap get ssid
	- net ap get psk
	- net ap get key_mgmt
	- net ap get wpa
	- net ap get rsn
	- net ap get proto
	- net ap get auth_alg
	- net ap get group_rekey
	- net ap get strict_rekey
	- net ap get gmk_rekey
	- net ap get ptk_rekey
	- net ap get hw_mode
	- net ap get 80211n
	- net ap get channel
	- net ap get beacon_int
	- net ap get dtim
	- net ap get max_num_sta

net ap enable
net ap disable

net ap sta num								# 获取当前连接AP的STA的数量。
net ap sta info <num>						# 获取当前连接AP的最多<num>个STA的信息。


net mode ap
net ap disable
net ap config xfAP-XR871
net ap enable

netcmd lmac parse_flags_set 0x7fff 0x7fff

--------------------------------------------------------------------------------
iperf commands

TCP RX
------
$ net iperf -s -p 5004 -t 30
> iperf.exe -f m -i 5 -c 192.168.51.100 -p 5004 -t 30

TCP TX
------
> iperf.exe -s -f m -i 5 -p 5004
$ net iperf -c 192.168.51.101 -p 5004 -t 30

UDP RX
------
$ net iperf -u -s -f m -i 1 -p 5003
> iperf.exe -u -f m -i 5 -c 192.168.51.100 -p 5002 -t 30 -b 10m

UDP TX
------
> iperf.exe -u -s -f m -i 5 -p 5002
$ net iperf -u -c 192.168.51.101 -p 5002 -t 30
