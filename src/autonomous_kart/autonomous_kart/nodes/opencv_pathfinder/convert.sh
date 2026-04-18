ffmpeg -fflags +genpts -i labeled_video.mp4 -c:v libx264 -preset fast -pix_fmt yuv420p -c:a aac -movflags +faststart fixed.mp4
