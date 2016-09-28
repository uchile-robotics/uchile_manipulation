#!/usr/bin/env python                                                                                                                                                                      
import subprocess
import time
import os


class VLCPlayBackground(object):
    """VLCPlayBackground"""
    def __init__(self):
        super(VLCPlayBackground, self).__init__()
        self.current_process = None

    def play(self, filename):
        self.kill()
        self.current_process = subprocess.Popen(
            ['vlc','--fullscreen','--repeat', filename], shell=False, 
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def wait(self):
        if self.current_process is not None:
            self.current_process.wait()

    def kill(self):
        if self.current_process is not None:
            self.current_process.kill()
        self.current_process = None


def get_video_files(folder, extensions=['mp4','m4v']):
    file_list=list()
    for video_file in os.listdir(folder):
        for ext in extensions:
            if video_file.endswith(ext) and not video_file.startswith('.'):
                file_list.append(os.path.join(folder, video_file))
    return file_list


if __name__ == '__main__':
    vlc = VLCPlayBackground()
    videos = get_video_files('~/video')
    for video in videos:
        vlc.play(video)
        time.sleep(5)
        vlc.kill()
