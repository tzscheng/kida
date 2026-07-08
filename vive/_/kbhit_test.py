import sys, tty, termios, select

def getch(timeout=0.01):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout) #timeout=0 => non-blocking
        if r: return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

CURSOR_HOME = "\033[H" # \033[H: 커서를 홈 위치(0, 0)로 이동
CLEAR_REST_OF_SCREEN = "\033[J" #\033[J: 커서 위치부터 화면 끝까지 지우기 (잔여 프롬프트 제거용)

cnt = 0
key = None
print('Press keys (q to quit)')

while True:
    c = getch(timeout=0.02)
    if c != None:
        key = c
        if c == 'q': break

    # do work
    sys.stdout.write(CURSOR_HOME)
    print('-------------------------------')
    print('cnt: %d                        ' %cnt)
    print('key: %s                        ' %key)
    print('-------------------------------')
    sys.stdout.write(CLEAR_REST_OF_SCREEN) 
    sys.stdout.flush()
    cnt += 1
