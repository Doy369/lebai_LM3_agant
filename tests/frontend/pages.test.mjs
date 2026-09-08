import test from 'node:test';
import assert from 'node:assert/strict';
import {PreviewConnection} from '../../docs/preview-connection.mjs';

test('Pages performs no fetch until explicitly enabled, and blocks again after disconnect', async () => {
  const calls = [];
  const connection = new PreviewConnection(async (...args) => {
    calls.push(args);
    return {ok:true, json:async () => ({success:true})};
  });
  await assert.rejects(connection.fetch('/api/robot/status'), /未连接后端/);
  assert.equal(calls.length, 0);
  connection.setEnabled(true);
  assert.deepEqual(await connection.fetch('/api/robot/status', {method:'GET'}), {success:true});
  assert.equal(calls[0][1].method, 'GET');
  connection.setEnabled(false);
  await assert.rejects(connection.fetch('/api/robot/status'), /未连接后端/);
  assert.equal(calls.length, 1);
});

test('disconnect rejects late responses even when the transport ignores abort', async () => {
  let finish;
  const connection = new PreviewConnection(() => new Promise(resolve => { finish = resolve; }));
  connection.setEnabled(true);
  const pending = connection.fetch('/api/robot/status');
  connection.setEnabled(false);
  connection.setEnabled(true);
  finish({ok:true, json:async () => ({success:true})});
  await assert.rejects(pending, /已取消/);
});

test('feedback timeout signal reaches the request and cannot produce a valid sample', async () => {
  let finish, requestSignal;
  const connection = new PreviewConnection((_url, options) => {
    requestSignal = options.signal;
    return new Promise(resolve => { finish = resolve; });
  });
  connection.setEnabled(true);
  const timeout = new AbortController();
  const pending = connection.fetch('/api/robot/status', {signal:timeout.signal});
  timeout.abort();
  assert.equal(requestSignal.aborted, true);
  finish({ok:true, json:async () => ({success:true})});
  await assert.rejects(pending, /已取消/);
});
