# NTRIP Client

network RTKを利用するためには，
NTRIP Server, NTRIP Caster, NTRIP Client
が必要である．
NTRIP ServerがGNSS基準局として修正情報を発行し，
NTRIP Casterがその情報をinternet上で中継し，
NTRIP Clientが移動局としてその情報を取得する．

本programではNTRIP Clientを動作させる．
多くの善意の基準局の情報が発行されている
NTRIP Casterとして有名なRTK2GOへTCPで接続して，
RTCM3.x protocolの情報を取得する．
これを1 packetとして認識できるように分割した後，
ros topicとして発行する．
