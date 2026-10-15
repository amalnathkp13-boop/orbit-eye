// Builds web/index.html: inlines the simulation engine and validation results into the page template.
const fs = require('fs'), p = require('path');
const t = fs.readFileSync(p.join(__dirname, 'web/template.html'), 'utf8');
const eng = fs.readFileSync(p.join(__dirname, 'sim/engine.js'), 'utf8');
const res = fs.readFileSync(p.join(__dirname, 'results/results.json'), 'utf8');
const body = t.replace('/*__ENGINE__*/', () => eng).replace('/*__RESULTS__*/', () => res);
const html = '<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n</head>\n<body>\n' + body + '\n</body>\n</html>\n';
fs.writeFileSync(p.join(__dirname, 'web/index.html'), html);
console.log('wrote web/index.html', html.length, 'bytes');
