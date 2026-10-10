"""Create explicitly labelled review/trial ZIPs, never a production release."""
import json
import argparse
import zipfile
from pathlib import Path

HERE=Path(__file__).resolve().parent


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--inputs',type=Path,default=HERE/'inputs.json');parser.add_argument('--output',type=Path,default=HERE/'output');parser.add_argument('--docs',type=Path,default=HERE)
    args=parser.parse_args();output=args.output.resolve();docs=args.docs.resolve();report=json.loads((output/'build_report.json').read_text())
    if report['status']!='DRAFT_NOT_RELEASED':raise ValueError('This packager only packages drafts')
    documents=[docs/n for n in ('README.md','MEASUREMENTS.md','SHOP_HANDOFF.md','FIT_TEST.md','ASSEMBLY.md','MACHINING.md')]+[args.inputs.resolve()]
    documents += [docs/n for n in ('THAI_PARTS_RESEARCH.md','WEB_CONFIRMED.md','web_confirmed.json','sources.json','hardware_source_map.json','PROCUREMENT.md','SUPPLIER_REQUESTS.md','procurement_inputs.json','procurement_report.json','VENDORS.md','vendors.json','PARTS_SELECTION.md','parts_selection.json','ELECTRICAL.md','CLOSEOUT.md','MASS_BALANCE.md','measurement_log.json','fit_log.json','quote_comparison.json') if (docs/n).exists()]
    if (docs/'RFQ').exists():documents+=list((docs/'RFQ').rglob('*'))
    if (docs/'evidence').exists():documents+=list((docs/'evidence').rglob('*'))
    reports=[output/n for n in ('STATUS.md','VERIFICATION.md','manifest.json','build_report.json','simulation.json','assembly_preview.png') if (output/n).exists()]
    prefix=report['revision'].replace('_PRINT_','_')
    for name,full in [(prefix+'_TRIAL_ONLY.zip',False),(prefix+'_DRAFT_REVIEW.zip',True)]:
        files=documents+reports+list((output/'TRIAL_ONLY').glob('*.stl'))
        if full:files+=list((output/'DRAFT').rglob('*'))
        with zipfile.ZipFile(output/name,'w',compression=zipfile.ZIP_DEFLATED) as archive:
            archive.writestr('START_HERE.txt',report['revision']+' — '+('DRAFT REVIEW' if full else 'TRIAL ONLY')+'\nNOT A PRODUCTION RELEASE.\nUnmeasured hardware and missing physical/shop evidence remain.\nFull project budget <=6000 THB; target mass <=1900g, hard limit <=2000g.\nRead '+str(output.relative_to(HERE)/'STATUS.md')+' and '+str(docs.relative_to(HERE)/'README.md')+' before printing.\nZIPs contain shop artifacts/docs; the source builder lives in the robot repository.\nBuild SHA256: '+report['build_sha256']+'\n')
            for path in files:
                if path.is_file():archive.write(path,path.relative_to(HERE))
        with zipfile.ZipFile(output/name) as archive:
            bad=archive.testzip()
            if bad:raise RuntimeError('Corrupt ZIP entry: '+bad)
        print(name,(output/name).stat().st_size,'bytes')


if __name__=='__main__':main()
