#!/usr/bin/ruby

# 気球追跡プログラム Ver. 1.11

# コマンドラインオプション ( 順序は自由 )
# init : habhub の予測情報を用いて位置を初期化する
# local : 位置情報をサーバに送らない

# 必要な入力ファイル
# habhub.kml : Habhub の予測ファイル(必須)
# retriever.txt : 追跡者の位置ファイル(必須)
#     経度,緯度,高度,速度,進行方位
# (例)138.56045,35.68215,0,1.2,187

# 出力ファイル ( 事前準備不要 )
# realtime.kml : Google earth で読み込むためのファイル

# その他の中間ファィルは自動生成される # 事前準備は不要
# 同名のファイルがあると init で上書きされる
# ワーキングディレクトリに無関係なものを置かないのが原則

require 'socket'
require 'time'
require 'serialport'
require 'fileutils'
include Math

# コマンドラインで local を指定するとサーバに送信しない。
# init を指定すると軌跡情報を消去し、Habhub 予測で初期化する。
#
local = init = false
ARGV.each{ |arg|
   case arg
     when 'local'
       local = true
     when 'init'
       init = true
   end
}

#Kikyu_serverIP = '133.23.4.15'
Kikyu_serverIP = '18.182.16.49'
Kikyu_serverPort = 5555
Station_id = '0055'
CurrentLoc_id = '00'

TrackFiles = [
  BalloonTrackFile = 'balloon-track.txt',
  RetrieverTrackFile = 'retriever-track.txt',
  PredictionTrackFile = 'prediction-track.txt',
  HabhubTrackFile = 'habhub-landing-track.txt'
]
if init
  FileUtils.rm( TrackFiles, {:force=>true} )
end
RetrieverPositionFile = 'retriever.txt'
RealtimeKmlFile = 'realtime.kml'
KmlFile = 'track.kml'
HabhubKmlFile = 'habhub.kml'
LogFile = 'flight-log.txt'
VoiceFile = 'voice.txt'

SerialPrefix = '/dev/tty.usbserial-'
ToCoSticks = ['MWF3K2Z', 'AHXDVQXS', 'AHXFIR8P', 'AHXDYI0Q', 'MWF607W']
#              16         18          20          22          26
Antenna_name = [ 'E: ', 'D: ', 'U: ' ]
antennas = []
ToCoSticks.each{ |fn|
  fullpath = SerialPrefix + fn 
  if File.exist?(fullpath)
    printf "%s is connected\n", fullpath
    sp = SerialPort.new(fullpath, 115200, 8, 1, 0) 
                        #device, rate, data, stop, parity
    sp.read_timeout = 50 # ms
    sp.flow_control = SerialPort::NONE
    antennas << sp
  end
}

if antennas.empty?
  print "No antenna device found.\n"
  exit
end

Format_p5nl = ( Format_p5 = "%.5f,%.5f,%.1f" ) + "\n"
NEWS = [ '  N', 'NNE', ' NE', 'ENE', '  E', 'ESE', ' ES', 'SSE', 
         '  S', 'SSW', ' SW', 'WSW', '  W', 'WNW', ' NW', 'NNW' ]
VNEWS = [ '北', 'ほーく北東', '北東', 'とう北東', 
          '東', 'とう南東', '南東', 'なん南東', 
         '南', 'なん南西', '南西', 'せい南西', 
         '西', 'せい北西', '北西', 'ほーく北西' ]

# 位置情報を管理するクラス Cordinate
# 気球の高度によって情報の粒度を変化させ、データ量を圧縮している。
# 高度が低い時は、粒度は細かくカバー範囲は狭い。高い時は逆。
#
class Cordinate

    attr_reader :falling, :altitude, :landing_speed, :ascent_rate
    Atmo_constant = 7500.0
    Std_Lnd_Spd = 4.5

    def initialize( lng, lat, alt, max_alt, lnd_lng, lnd_lat, 
                    lnd_speed = Std_Lnd_Spd ) 

        @init_time = @time = Time.now
	@init_alt = @altitude = alt
        @max_altitude = [ max_alt, @altitude ].max
        @ialt = case
          when alt < 8000.0 
            ( alt * 0.016 ).to_i
          when alt < 16000.0
            0x80 | ( ( alt -  8000.0 ) * 0.008 ).to_i
          else
            0xc0 | ( ( alt - 16000.0 ) * 0.002 ).to_i
        end
	@i_lat = ( lat * ( 1<<16 ) ).to_i
	@i_lng = ( lng * ( 1<<16 ) ).to_i

        @falling = false
        @landing_speed = lnd_speed

        @pred_dir, @pred_dist, = direction( lnd_lng, lnd_lat, lng, lat )
        @pred_dir *= PI / 180.0
        @i_dir  = ( ( ( @pred_dir/360.0 )*256 ).round ) & 0xff
        @pred_dist *= @landing_speed / Std_Lnd_Spd
        @i_dist = ( @pred_dist*0.16/( [@ialt,16].max ) ).round
        
    end

    def self.alt_decode( ialt )
        case
	    when  ( ialt & 0xc0 ) == 0xc0 
		( ialt & 0x3f ) * 1000.0/2.0 + 16000.0
	        # 鉛直粒度
		# 高度 16km 以上: 500m 刻みで、47.5km まで
	    when  ( ialt & 0x80 ) == 0x80 
		( ialt & 0x7f ) * 1000.0/8.0 + 8000.0
		# 高度 8km 以上: 125m 刻みで 16km まで
  	    else
		ialt * 1000.0/16.0
		# 高度 8km 以下: 62.5m 刻み
	end
    end

    def adjust( curr, update, span , shift )
	new = ( curr & ( ( - span ) | ( ( 1 << shift ) - 1 ) ) ) \
               | ( ( update << shift ) & ( span - 1 ) )
        half = span >> 1
        if new > curr 
	    if new - curr > half 
		new -= span
	    end
	else
	    if curr - new > half 
		new += span
	    end
	end
	new
    end

    def update( alt, lat_14, lng_14, t = Time.now )

        @time = t
        @ialt = alt

        @altitude = Cordinate.alt_decode( alt )
        @max_altitude = [ @max_altitude, @altitude ].max

	# 水平粒度
	shift = case
            when @altitude < 2000.0 
		0   # 高度 2km 以下: 範囲 1/4度(約20km) 粒度 約1.7m
	    when @altitude < 4000.0 
		2   # 高度 4km 以下: 範囲 1度(約80km)   粒度 約7m
	    else		 	  
		5   # 高度 4km 以上: 範囲 8度(約650km)  粒度 約55m
	end

	span = ( 1 << ( 14 + shift ) );
	@i_lat = adjust( @i_lat, lat_14, span, shift );
	@i_lng = adjust( @i_lng, lng_14, span, shift );

    end
    def latitude
        @i_lat.to_f / (1<<16)
    end
    def longitude
        @i_lng.to_f / (1<<16)
    end

    def adjust_prefix( prev, new, bits )
       suffix_mask = ( 1 <<  bits ) - 1
       prefix_mask = ~suffix_mask
       prefix = prev & prefix_mask
       range = 1 << bits
       half_range = range >> 1
       diff = ( new & suffix_mask ) - ( prev & suffix_mask ) 
       prefix += range if diff < - half_range 
       prefix -= range if diff > half_range 
       prefix | ( new & suffix_mask )
    end
    def update_pred( prediction )
       case ( ( Time.now.sec + 3 ) / 10 ).truncate % 3
          when 1 
            @pred_dir = prediction/256.0 * 2.0 * PI
            # clockwise from the north
            # printf "Dir: %d %f\n", prediction, @pred_dir
          when 0 
            @falling = ( prediction & 0x80 ) == 0x80  
            speed = ( prediction & 0x7f ) /10.0
            if @falling
              @landing_speed = speed
            else
              @ascent_rate = speed if @altitude > 2000.0
            end
            #printf "Rate: %d %s %f %f\n", 
            #  prediction, @falling, ( @ascent_rate ? @ascent_rate : 0 ), 
            #  ( @landing_speed ? @landing_speed : 0 )
          when 2
            @i_dist = [ adjust_prefix( @i_dist, prediction, 8 ), 0 ].max
            @pred_dist = @i_dist * [@ialt, 16].max / 0.16
            # printf "Dist: %d %d %f\n", prediction, @i_dist, @pred_dist
       end
    end

    def scaled_dist
       @falling ? @pred_dist : @pred_dist * Std_Lnd_Spd / @landing_speed
    end

    def pred_lat
        self.latitude + self.scaled_dist * cos(@pred_dir)/111111.0
    end
    def pred_lng 
        self.longitude + self.scaled_dist * sin(@pred_dir)/111111.0
    end
    def time_to_land
        2.0 * Atmo_constant / @landing_speed * 
                 ( 1.0 - exp( - @altitude / ( 2.0 * Atmo_constant ) ) )
    end
    def output( stream )
       rate = self.ascent_rate ? "%4.1fm/s" % self.ascent_rate : ""
       stream.printf @time.strftime('%H:%M:%S ')
       stream.printf "loc: %5.5f %5.5f  ", self.longitude, self.latitude
       stream.printf "alt: %5.0f m %s\n", @altitude, rate
       stream.printf "Lands in: %5.1f min around %5.3f %5.3f %4.1fm/s\n", 
                     (self.time_to_land/60), self.pred_lng, self.pred_lat,
                     @landing_speed
    end
end # Class Cordinate

def direction( target_lng, target_lat, base_lng, base_lat, 
               target_alt = 0.0,  base_alt = 0.0 )

    x = ( target_lng - base_lng ) * 111111 * cos( base_lat * PI / 180.0) 
    y = ( target_lat - base_lat ) * 111111
    azim = atan2( x, y ) * 180.0 / PI # angle from the north
    azim += 360.0 if azim < 0

    hdist = sqrt( x*x + y*y )
    z = target_alt - base_alt
    elev = atan2( z, hdist ) * 180.0 / PI

    return azim, hdist, elev
end

#
# Habhub の予測軌道を扱うクラス
#
class Habhub

    attr_reader :planned_landing_spd, :planned_ascent_rate

    def initialize( habhub_file )
      @coords = []
      File.open( habhub_file, 'r') do |f| 
        coord_sec = false
        f.each { |line| 
          break if line =~ /<\/coordinates>/
          if line =~ /Ascent/ 
             fields = line.split(/:\s*|m\/s/)
             # p fields
             printf "Habhub ascent %.2fm/s, landing %.2fm/s\n", 
                    @planned_ascent_rate = fields[1].to_f,
                    @planned_landing_spd = fields[3].to_f
          end
          if line =~ /<coordinates>/
            coord_sec = true
            next
          end
          @coords.push line.split(/\s*,\s*/).map{ |x| x.to_f }  if coord_sec
        }
      end
      @coords.freeze
      @Max_alt_i = @coords.each_with_index.max_by{ |x,i| x[2] }[1]
      @Max_alt = @coords[@Max_alt_i][2]
      @index = 0
    end

    def find_start( altitude = 0.0, climb_p = true )

      @index = [@index, @Max_alt_i].max unless climb_p
      if climb_p and altitude < @Max_alt
       @index += 1 while @index < @coords.size and @coords[@index][2] < altitude
      else
       @index += 1 while @index < @coords.size and @coords[@index][2] > altitude
      end
      @index = [@index, @coords.size-1].min
      @coords[@index]

    end

    def prediction( altitude = 0.0, climb_p = true, speed = nil )

      ascent_scale = 1.0
      descent_scale  = 1.0
      if climb_p
         ascent_scale = @planned_ascent_rate/speed if speed
      else
         descent_scale = @planned_landing_spd/speed if speed 
      end

      lng_0, lat_0, = self.find_start( altitude, climb_p )
      res = @coords[@index..@Max_alt_i].map{ |x| 
        [ (x[0] - lng_0)*ascent_scale, (x[1] - lat_0)*ascent_scale, x[2] ] }
      lng_off, lat_off, = res.empty? ? [ 0.0, 0.0 ] : res[-1]
      lng_0, lat_0, = @coords[ [@Max_alt_i,@index].max ]
      res.concat( @coords[ [@Max_alt_i,@index].max + 1 .. -1].map{ |x|
             [ (x[0] - lng_0)*descent_scale + lng_off, 
               (x[1] - lat_0)*descent_scale + lat_off, x[2] ] } )
      res.empty? ? [[0.0, 0.0, 0.0]] : res
    end
end # Class Habhub

def location_msg( id, lat, lng, height, type )

    lat_deg = lat.truncate
    lat_min = ( ( lat - lat_deg ) * 60 * 10000 ).truncate
    lng_deg = lng.truncate
    lng_min = ( ( lng - lng_deg ) * 60 * 10000 ).truncate

    location = sprintf( "%02X%02d%06d%02X%04d%06d%06d", \
       ?N.ord, lat_deg, lat_min, ?E.ord, lng_deg, lng_min, height.truncate )

    time = Time.now.strftime("%Y%m%d%H%M%S")

    msg = id + time + type + location
    [msg].pack("H*")
end

def checksum_nibble( data )
    cs = data.inject { |cs,x| cs ^= x }
    ( cs >> 4 ) ^ ( cs & 0x0f ) == 0
end

class VoiceOver
  def initialize( file = STDOUT )
     @voice_text = ""
     @voice_file = file
  end 
  def append text
     @voice_text <<  text
  end
  def prepend text
     @voice_text = text + @voice_text
  end
  def flush ( text = "", pause = 0.0 )
    self.append( text )
    File.open( @voice_file, 'a' ) do |f|
      f.printf "\n %s \n\n", @voice_text 
      @voice_text.clear
    end
    sleep( pause )
  end
end

#
# main
#

unless local
   udpc = UDPSocket.new
   udpc.connect( Kikyu_serverIP, Kikyu_serverPort )
end

# Habhub pre-flight prediction
if File.exist?( HabhubKmlFile ) 
  habhub = Habhub.new( HabhubKmlFile )
else
  print HabhubKmlFile + " doesn't exist\n"
  exit
end

File.open( RealtimeKmlFile, 'w' ) do |f|
  f.print( kml_realtime().sub( /%KmlFile%/, KmlFile ) )
end

if File.exist?( BalloonTrackFile )  # ログファイルに基づいて初期値を設定
  max_alt = 0.0
  lng, lat, alt = 0.0, 0.0, 0.0
  File.open( BalloonTrackFile, 'r' ) do |f|
    f.each { |line|
      coords = line.split(/\s*,\s*/)
      if coords.size >= 3 
        lng, lat, alt = coords.map{ |s| s.to_f }
        max_alt = [ max_alt, alt ].max
      end
    }
  end
  lnd_lng, lnd_lat, lnd_speed = lng + 0.01, lat, habhub.planned_landing_spd
  if File.exist?( PredictionTrackFile ) 
    line3f = ''
    File.open( PredictionTrackFile, 'r' ) do |f|
      f.each { |line| 
        line3f = line if line.split(/,/).size >= 3  
      }
    end
    lnd_lng, lnd_lat, lnd_speed = line3f.split(/\s*,\s*/).map{ |s| s.to_f }
  end
  balloon = Cordinate.new( lng, lat, alt, max_alt, lnd_lng, lnd_lat, lnd_speed )

end

voice = VoiceOver.new( VoiceFile )

counter = 0
log_string = ""

while true

  lines = []
  antennas.each_with_index{ |ant, i|
     line = ant.read
     if line.size > 0
        lines[i] = line
        break
     end
  }
  antennas.each_with_index{ |ant, i|
     unless lines[i]
        line = ant.read
        if line.size > 0
           lines[i] = line
        end
     end
  }

  if lines.size == 0 
     sleep 1.0
     next if ( counter += 1 ) < 10
  end

  if lines.size > 0 

    now = Time.now

    data = ""
    src = nil
    lines.each_with_index{ |line, i|
       if line 
          print s = Antenna_name[i]
          log_string << s
          voice.prepend( s )
          comma = line.index(',')
          comma = line.index(',', comma + 1 ) if comma
          comma = line.index(',', comma + 1 ) if comma
          if comma 
             src, src1, rssi = line[0..comma].split(',').map{ |s| s.to_i }
             data = line[comma+1..-1].unpack('C*')
             print s = "%3d " % rssi
             log_string << s
             voice.prepend( " %3ddB " % -rssi  )
          else
             data = line.unpack('C*')
          end
       end
    }
    if src
       voice.append( " %d : " % src ) 
       print s = " From %d " % src
       log_string << s
    end

    if data.size < 5
       printf "Too short frame: %d\n", data.size
       next
    end

    if ! checksum_nibble( data[0,5] )
       printf "Checksum error! \n"
       next
    end

    print s = "\n"
    log_string << s

    ialt = data[0].to_i
    lat_14 = ( data[1].to_i << 6 ) | 
       ( ( data[3] & 0xf0 ) >> 2 ) | ( ( data[4] & 0xc0 ) >> 6 )
    lng_14 = ( data[2].to_i << 6 ) | 
       ( ( data[3] & 0x0f ) << 2 ) | ( ( data[4] & 0x30 ) >> 4 )

    unless balloon  # Habhub の予測に基づいて初期値を設定
      altitude = Cordinate.alt_decode( ialt )
      launch = habhub.find_start( )
      start = habhub.find_start( altitude )
      balloon = Cordinate.new( start[0], start[1], altitude, altitude,
        start[0]*2-launch[0], start[1]*2-launch[1], habhub.planned_landing_spd )
    end

    # 気球の位置更新
    balloon.update( ialt, lat_14, lng_14, now )

    File.open( BalloonTrackFile, 'a') do |f|
       f.printf( Format_p5nl,
                balloon.longitude, balloon.latitude, balloon.altitude )
    end

    # 予想着地点の更新
    if data.size >= 6 
      balloon.update_pred( data[5].to_i )
      File.open( PredictionTrackFile, 'a') do |f|
        f.printf( Format_p5nl,
              balloon.pred_lng, balloon.pred_lat, balloon.landing_speed )
      end
    end

    balloon.output( STDOUT )
    File.open( LogFile, 'a' ) do |f|
      f.print log_string
      log_string.clear
      balloon.output( f )
    end

    unless local # サーバに送信
       udpc.send( location_msg( 
             Station_id, balloon.latitude, balloon.longitude,
             balloon.altitude, CurrentLoc_id ),        0 )
    end

  end

  retriever_lng, retriever_lat, retriever_alt, speed, heading = 
     File.open( RetrieverPositionFile, 'r') do |f|
        f.read.split(/\s*,\s*/).map{ |s| s.to_f }
     end
  File.open( RetrieverTrackFile, 'a') do |f|
    f.printf("%1.5f,%1.5f,%1.1f\n", retriever_lng, retriever_lat, heading )
  end

  next unless balloon

  azim, hdist, elev =
    direction( balloon.longitude, balloon.latitude, 
             retriever_lng, retriever_lat, balloon.altitude, retriever_alt )

  iazim = ( azim/22.5 + 0.5 ).truncate % 16

  clockpos = ( azim - heading ) /30.0
  clockpos += 12.0 if clockpos < 0
  clockpos -= 12.0 if clockpos >= 12.0
 
  counter = 0
  print s = "%s az:%3.0f clkpos:%4.1f elv:%2.0f %6.0fm\n" % 
             [ NEWS[iazim], azim, clockpos,  elev, hdist ]
  log_string << s

  # Voice output
  clockpos += 0.25
  h = clockpos.truncate
  m30 = ( ( clockpos - h ) * 2 ).truncate
  voice.flush( "%dm %s方向 %d時 %s : %.0f度 %.0fkm" % 
   [ balloon.altitude, VNEWS[iazim], h, m30>0 ? "半":"", elev, hdist/1000.0 ] )

  # kmlファイルの作成

  kml = kml_template

  # Balloon
  kml['%balloon%'] = "Alt:%d %s:%.1f Elv:%.1f" %
           [ balloon.altitude, NEWS[iazim], clockpos, elev ] 
  kml['%balloon-coord%'] = Format_p5 %
           [ balloon.longitude, balloon.latitude, balloon.altitude ] 
  File.open( BalloonTrackFile, 'r') do |f|
    kml['%balloon-track%'] = f.read
  end

  # Prediction
  kml['%prediction%'] = "Land in %.1fmin. %.1fm/s" %
           [ balloon.time_to_land/60, balloon.landing_speed ] 
  kml['%prediction-coord%'] = Format_p5 %
           [ balloon.pred_lng, balloon.pred_lat, 0.0 ] 
  File.open( PredictionTrackFile, 'r') do |f|
    kml['%prediction-track%'] = f.read
  end

  # Habhub prediction
  track = habhub.prediction( balloon.altitude, ! balloon.falling, 
          balloon.falling ? balloon.landing_speed : balloon.ascent_rate ).
     map{ |x| [ x[0]+balloon.longitude, x[1]+balloon.latitude, x[2] ] }
  kml['%habhub%'] = track.inject(""){ |s,l| s.concat( Format_p5nl % l )}
  File.open( HabhubTrackFile, 'a+') do |f|
    kml['%habhub-landing-track%'] = f.read + ( Format_p5nl % track[-1] )
    f.printf( Format_p5nl % track[-1] )
  end

  # Retriever
  kml['%retriever%'] = "Retriever" 
  kml['%retriever-coord%'] = Format_p5 %
           [ retriever_lng, retriever_lat, retriever_alt ] 
  kml['%retriever-direction%'] = "%d" % ( ( heading/22.5 + 0.5 ).to_i & 0x0F )
  File.open( RetrieverTrackFile, 'r') do |f|
    kml['%retriever-track%'] = f.read
  end

  File.open( KmlFile, 'w') do |f|
    f.print( kml )
  end

end

# KML テンプレート

BEGIN{

def kml_template 

<<EOF
<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>  
  <name>Balloon Tracker</name> <open>1</open>  
  <Style id="downArrowIcon">    
    <IconStyle>      
      <Icon>        
        <href>http://maps.google.com/mapfiles/kml/shapes/arrow.png</href>
      </Icon>    
    </IconStyle>  
  </Style>  
  <Style id="balloonIcon">    
    <IconStyle>      
      <Icon>        
        <href>http://maps.google.com/mapfiles/kml/paddle/wht-blank.png</href>
      </Icon>    
    </IconStyle>  
  </Style>  
  <Style id="TrackIcon">    
    <IconStyle>      
      <Icon>        
        <href>http://earth.google.com/images/kml-icons/track-directional/track-%retriever-direction%.png</href>
      </Icon>    
    </IconStyle>  
  </Style>  
  <Style id="greenLineGreenPoly">    
    <LineStyle> <color>7f00ff00</color> <width>4</width> </LineStyle>  
    <PolyStyle> <color>7f00ff00</color> </PolyStyle>  
  </Style>  
  <Style id="orangeLineGreenPoly">    
    <LineStyle> <color>7f007fff</color> <width>4</width> </LineStyle>  
    <PolyStyle> <color>7f00ff00</color> </PolyStyle>  
  </Style>  
  <Style id="yellowLineGreenPoly">    
    <LineStyle> <color>7f00ffff</color> <width>4</width> </LineStyle>  
    <PolyStyle> <color>7f00ff00</color> </PolyStyle>  
  </Style>  
  <Style id="transPurpleLineGreenPoly">  
    <LineStyle> <color>7fff00ff</color> <width>4</width> </LineStyle>  
    <PolyStyle> <color>7f00ff00</color> </PolyStyle>  
  </Style>  
  <Folder>  
    <name>Balloon Tracking</name>  <open>1</open>  
    <Placemark> <name>%balloon%</name> <styleUrl>#balloonIcon</styleUrl>  
      <visibility>1</visibility>  
      <Point>   
        <altitudeMode>absolute</altitudeMode>    
        <coordinates>%balloon-coord%</coordinates>  
      </Point>  
    </Placemark>  
    <Placemark> <name>%prediction%</name> <styleUrl>#downArrowIcon</styleUrl>  
      <visibility>1</visibility>  
      <Point>   
        <altitudeMode>clampToGround</altitudeMode>    
        <coordinates>%prediction-coord%</coordinates>  
      </Point>  
    </Placemark>  
    <Placemark> <name>%retriever%</name> <styleUrl>#TrackIcon</styleUrl>  
      <visibility>1</visibility>  
      <Point>   
        <altitudeMode>clampToGround</altitudeMode>    
        <coordinates>%retriever-coord%</coordinates>  
      </Point>  
    </Placemark>  
    <Placemark> <name>balloon</name> <styleUrl>#yellowLineGreenPoly</styleUrl>  
      <LineString> 
        <extrude>1</extrude> <altitudeMode>absolute</altitudeMode> 
        <coordinates>%balloon-track%</coordinates>      
      </LineString>   
    </Placemark>  
    <Placemark> <name>retriever</name> 
      <styleUrl>#transPurpleLineGreenPoly</styleUrl>      
      <LineString> 
        <tessellate>1</tessellate> <altitudeMode>clampToGround</altitudeMode>   
        <coordinates>%retriever-track%</coordinates>      
      </LineString>   
    </Placemark>  
    <Placemark> <name>landing</name> <styleUrl>#greenLineGreenPoly</styleUrl>  
      <LineString>          
        <tessellate>1</tessellate> <altitudeMode>clampToGround</altitudeMode>   
        <coordinates>%prediction-track%</coordinates>      
      </LineString>   
    </Placemark>  
    <Placemark> <name>Habhub</name> <styleUrl>#greenLineGreenPoly</styleUrl> 
      <LineString>          
        <extrude>1</extrude> <altitudeMode>absolute</altitudeMode>          
        <coordinates>%habhub%</coordinates>      
      </LineString>   
    </Placemark>  
    <Placemark> <name>Habhub Landing</name> 
      <styleUrl>#orangeLineGreenPoly</styleUrl>      
      <LineString>          
        <tessellate>1</tessellate> <altitudeMode>clampToGround</altitudeMode>   
        <coordinates>%habhub-landing-track%</coordinates>      
      </LineString>   
    </Placemark>  
  </Folder> 
</Document>
</kml>
EOF

end

def kml_realtime 

<<EOF
<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<NetworkLink>
	<name>Realtime</name>
	<open>1</open>
	<Link>
		<href>%KmlFile%</href>
		<refreshMode>onInterval</refreshMode>
		<refreshInterval>30</refreshInterval>
	</Link>
</NetworkLink>
</kml>
EOF
end

}
