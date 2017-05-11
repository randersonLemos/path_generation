ffmpeg -f image2 -r 30 -i %05d.png -vcodec mpeg4 -b 3000k -y movie.mp4
ffmpeg -f image2 -r 30 -i %05d.png -vcodec mpeg4 -b 1500k -y movie.mp4
