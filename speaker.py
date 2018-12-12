import sh
import thread

class Speaker:

    SPEAKER_LIST = ['hongsiqi', 'linmeixia']
    
    def __init__(self, who='hongsiqi'):
        self.who = who

    def mpg321(self, mp3):
        sh.mpg321(mp3)

    def say(self, what, multi_thread=True):
        if isinstance(what, str):
            mp3 = 'audio/{}/{}.mp3'.format(self.who, what)
            if multi_thread:
                thread.start_new_thread(self.mpg321, (mp3,))
            else:
                self.mpg321(mp3)


if __name__ == '__main__':
    sp = Speaker()
    import ipdb; ipdb.set_trace()