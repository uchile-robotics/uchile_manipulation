#!/usr/bin/env python                                                                                                                                                                      
import subprocess
import time
import os


class VLCPlayBackground(object):
    """VLCPlayBackground"""
    def __init__(self):
        super(VLCPlayBackground, self).__init__()
        self.current_process = None
        self._check_vlc()

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

    def _check_vlc(self):
        try:
            devnull = open(os.devnull)
            subprocess.Popen(['vlc', '--version'], stdout=devnull, stderr=devnull).communicate()
        except OSError as e:
            if e.errno == os.errno.ENOENT:
                raise RuntimeError('VLC is not installed')
            else:
                raise RuntimeError('Something wrong with VLC')
        except Exception:
            raise RuntimeError('Something wrong with VLC')


def get_video_files(folder, extensions=['mp4','m4v']):
    file_list=list()
    try:
        raw_list = os.listdir(folder)
    except OSError:
        raise RuntimeError('Folder \'{}\' not found'.format(folder))
    for video_file in raw_list:
        for ext in extensions:
            if video_file.endswith(ext):
                file_list.append(os.path.join(folder, video_file))
    file_list.sort()
    return file_list


if __name__ == '__main__':
    vlc = VLCPlayBackground()
    videos = get_video_files('/home/robotica/video', extensions=['mp4','avi'])
    for video in videos:
        vlc.play(video)
        time.sleep(5)
        vlc.kill()
