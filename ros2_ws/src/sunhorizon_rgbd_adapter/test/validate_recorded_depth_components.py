#!/usr/bin/env python3
"""Replay committed expectations against external raw evidence; no ROS needed."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary',type=Path,required=True)
    p.add_argument('--recording-dir',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    a=p.parse_args();a.output_dir.mkdir(parents=True,exist_ok=False)
    expected=json.loads(Path(__file__).with_name('recorded_depth_components.json').read_text())
    results=[]
    for frame in expected['frames']:
        raw=a.recording_dir/frame['file'];mask=a.output_dir/(frame['file']+'.mask')
        assert hashlib.sha256(raw.read_bytes()).hexdigest()==frame['input_sha256'],raw
        command=[str(a.binary.resolve()),str(raw),str(expected['width']),str(expected['height']),
                 str(expected['step']),'1' if expected['bigendian'] else '0',
                 str(expected['delta_m']),str(expected['minimum_pixels']),str(mask)]
        observed=json.loads(subprocess.check_output(command,text=True))
        for key in ('valid_pixels','removed_pixels','removed_components'):
            assert observed[key]==frame[key],(frame['file'],key,observed[key],frame[key])
        assert hashlib.sha256(mask.read_bytes()).hexdigest()==frame['mask_sha256'],frame['file']
        results.append({'file':frame['file'],**observed})
    (a.output_dir/'validation.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results,indent=2))


if __name__=='__main__':main()
