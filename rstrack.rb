#!/usr/bin/ruby

# Radio Sonde Tracking program

# required files
#   habhub.kml : predicted path of Habhub
#   retriever.txt : potsition and heading of retriever
#   (ex.)138.56045,35.68215,0,1.2,187

# output file
# realtime.kml : for Google earth

require 'fileutils'
require 'time'
require 'timeout'
include Math

# init option reases all paths
#
init = false
ARGV.each{ |arg|
   case arg
     when 'init'
       init = true
   end
}

TrackFiles = [
  BalloonTrackFile = 'balloon-track.txt',
  RetrieverTrackFile = 'retriever-track.txt',
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

Format_p5nl = ( Format_p5 = "%.5f,%.5f,%.1f" ) + "\n"
NEWS = [ '  N', 'NNE', ' NE', 'ENE', '  E', 'ESE', ' ES', 'SSE', 
         '  S', 'SSW', ' SW', 'WSW', '  W', 'WNW', ' NW', 'NNW' ]
VNEWS = [ '北', 'ほーく北東', '北東', 'とう北東', 
          '東', 'とう南東', '南東', 'なん南東', 
         '南', 'なん南西', '南西', 'せい南西', 
         '西', 'せい北西', '北西', 'ほーく北西' ]

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
end # Class VoiceOver

class Vertical 
  def initialize( alt = 0.0 )
     @prev_alt = alt
  end 
  def update( new_alt )
     alt = @prev_alt
     @prev_alt = new_alt
     return new_alt - alt
  end
end

def parse line

#            for rs41              for iMS-100
  if line =~ /\[00000\]/ or line =~ /\(ok\)\[OK\]/
     return line.partition( /lat:/ )[2].to_f,
            line.partition( /lon:/ )[2].to_f,
            line.partition( /alt:/ )[2].to_f,
            line.partition( /vH:/ )[2].to_f,
            line.partition( /D:/ )[2].to_f,
            line.partition( /vV:/ )[2].to_f
  else
     return nil
  end

end

def dist_string d
  return "%dキロメートル" % (d/1000.0).round if d >= 100000.0
  return "%.1fキロメートル" % (d/1000.0) if d >= 10000.0
  return "%dメートル" % (d.round(-1)) if d >= 1000.0
  return "%dメートル" % (d.round)
end

#
# main
#

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

last_log_line = nil
if File.exist?( LogFile )
  File.open( LogFile, "r" ){ |f|
    f.each_line{ |line|
      if line =~ / lon: /
        last_log_line = line
      end
    }
  }
end

voice = VoiceOver.new( VoiceFile )
prev_time = Time.now
Interval = 10
updated = false
v_v = Vertical.new( 0.0 )
lat = lon = alt = nil

while true

  line = ""
  begin
    Timeout.timeout(Interval) {
      line = STDIN.gets
    }
  rescue Timeout::Error
  end

  if line == "" and ! lon and last_log_line
    line = last_log_line
  end

  if line and line.size > 0 

    print line
    File.open( LogFile, 'a' ) do |f|
      f.print line
    end

    lat, lon, alt, hv, dir, vv  = parse line
	  
    next unless lat

    if vv == 0.0 
      vv = v_v.update( alt );
    end
	  
    File.open( BalloonTrackFile, 'a') do |f|
       f.printf( Format_p5nl, lon, lat, alt )
    end

    updated = true

  end # line.size > 0

  t = Time.now
  next if t - prev_time < Interval
  prev_time = t

  retriever_lng, retriever_lat, retriever_alt, speed, heading = 
     File.open( RetrieverPositionFile, 'r') do |f|
        f.read.split(/\s*,\s*/).map{ |s| s.to_f }
     end
  File.open( RetrieverTrackFile, 'a') do |f|
    f.printf("%1.5f,%1.5f,%1.1f\n", retriever_lng, retriever_lat, heading )
  end

  next unless lat

  azim, hdist, elev =
    direction( lon, lat, retriever_lng, retriever_lat, alt, retriever_alt )

  iazim = ( azim/22.5 + 0.5 ).truncate % 16

  clockpos = ( azim - heading ) /30.0
  clockpos += 12.0 if clockpos < 0
  clockpos -= 12.0 if clockpos >= 12.0
 
  print s = "%s az:%3.0f clkpos:%4.1f elv:%2.0f %6.0fm\n" % 
             [ NEWS[iazim], azim, clockpos,  elev, hdist ]
  File.open( LogFile, 'a' ) do |f|
      f.print s
  end

  # Voice output
  clockpos += 0.25
  h = clockpos.truncate
  m30 = ( ( clockpos - h ) * 2 ).truncate
  voice.append( "%dm " % [ alt.round(-2) ] ) if updated
  updated = false
  voice.flush( "%s方向 %d時 %s : %.0f度 %s" % 
             [ VNEWS[iazim], h, m30>0 ? "半":"", elev, dist_string(hdist) ] )

  # creating a kml file

  kml = kml_template

  # Balloon
  kml['%balloon%'] = "Alt:%d %s:%.1f Elv:%.1f" %
           [ alt, NEWS[iazim], clockpos, elev ] 
  kml['%balloon-coord%'] = Format_p5 % [ lon, lat, alt ] 
  File.open( BalloonTrackFile, 'r') do |f|
    kml['%balloon-track%'] = f.read
  end

  exp_m_alt = exp( - alt / ( 2.0 * 7500.0 ) )
  landing_speed = -vv * exp_m_alt
  time_to_land = 2.0 * 7500.0 / landing_speed * ( 1.0 - exp_m_alt )

  # Habhub prediction
  # track = habhub.prediction( alt, !falling, vv : landing_speed ).
  track = habhub.prediction( alt, vv > 0 ).
          map{ |x| [ x[0]+lon, x[1]+lat, x[2] ] }
  kml['%habhub%'] = track.inject(""){ |s,l| s.concat( Format_p5nl % l )}
  File.open( HabhubTrackFile, 'a+') do |f|
    kml['%habhub-landing-track%'] = f.read + ( Format_p5nl % track[-1] )
    f.printf( Format_p5nl % track[-1] )
  end
  # Prediction
  kml['%prediction%'] = "Land in %.1fmin. %.1fm/s" %
           [ time_to_land/60, landing_speed ] 
  kml['%prediction-coord%'] = Format_p5 % track[-1]
  kml['%prediction-track%'] = ""

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

# KML template

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
