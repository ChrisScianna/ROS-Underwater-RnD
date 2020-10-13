
def textindent(text, offset=4):
    return ''.join([' ' * offset + line for line in text.splitlines(True)])
