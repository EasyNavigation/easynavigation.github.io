// Copyright 2026 Intelligent Robotics Lab
// Banner on the development and old docs: the site is /<version>/..., versions in /versions.json
(function () {
  'use strict';

  var parts = window.location.pathname.split('/');
  var current = parts[1];
  var page = parts.slice(2).join('/');

  // Opens the same page in another version, or its home page if it does not exist there
  function go(version) {
    var target = '/' + version + '/' + page;
    fetch(target, {method: 'HEAD'}).then(function (r) {
      window.location.href = (r.ok ? target : '/' + version + '/') + window.location.hash;
    }).catch(function () {
      window.location.href = '/' + version + '/';
    });
  }

  function banner(text, version, title) {
    var body = document.querySelector('.md-content__inner');
    if (!body || body.querySelector('.en-version-banner')) {return;}
    var note = document.createElement('div');
    note.className = 'admonition warning en-version-banner';
    var p = document.createElement('p');
    p.appendChild(document.createTextNode(text + ' '));
    var link = document.createElement('a');
    link.href = '#';
    link.textContent = 'Go to ' + title + '.';
    link.addEventListener('click', function (e) {e.preventDefault(); go(version);});
    p.appendChild(link);
    note.appendChild(p);
    body.insertBefore(note, body.firstChild);
  }

  function show() {
    fetch('/versions.json').then(function (r) {return r.json();}).then(function (data) {
      var titles = {};
      data.versions.forEach(function (v) {titles[v.name] = v.title || v.name;});
      if (!(current in titles)) {return;}  // Not a versioned page (e.g. a local build)
      var latestRelease = titles[data.default];
      if (current === 'latest') {
        banner('This is the development documentation: it may describe features not released yet.',
          data.default, 'the documentation of the latest release (' + latestRelease + ')');
      } else if (current !== data.default) {
        banner('This is the documentation of EasyNav ' + titles[current] +
          ', not the latest release (' + latestRelease + ').', data.default, latestRelease);
      }
    }).catch(function () {});
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', show);
  } else {
    show();
  }
})();
