#!/usr/bin/env python3
"""Package unmodified Gazebo captures and reference into a standalone HTML review."""
import argparse
import base64
import html
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    def figure(title, path):
        content = base64.b64encode(path.read_bytes()).decode()
        mime = 'image/jpeg' if path.suffix.lower() in ('.jpg', '.jpeg') else 'image/png'
        return f'<figure><figcaption>{html.escape(title)}</figcaption><img src="data:{mime};base64,{content}"></figure>'
    views = [('Target reference (Epson; Panda retained in implementation)', args.reference),
             ('Panda parcel cell: initial layout', args.evidence / 'overview.png'),
             ('Panda parcel cell: completed batch', args.evidence / 'after/overview.png'),
             ('Completed batch: side view', args.evidence / 'after/side.png'),
             ('Functional overhead RGBD view', args.evidence / 'after/rgbd.png')]
    document = ('<!doctype html><meta charset="utf-8"><title>Panda parcel cell review</title>'
        '<style>body{background:#171b20;color:#eee;font:16px system-ui;margin:24px}'
        'main{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:20px}'
        'figure{margin:0}img{width:100%;height:auto}figcaption{padding:10px 0}'
        '@media(max-width:850px){main{grid-template-columns:1fr}}</style>'
        '<h1>Panda industrial parcel cell</h1><p>Unmodified reference and Gazebo camera captures. '
        'Dimensions adapted to Panda reach and gripper opening; the right feeder is stationary.</p><main>'
        + ''.join(figure(title, path) for title, path in views) + '</main>')
    args.output.write_text(document)
    print(args.output)


if __name__ == '__main__':
    main()
