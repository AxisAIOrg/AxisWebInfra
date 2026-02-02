function refreshNameBuffer(demo) {
  if (!demo.model) { return; }
  demo.namesBuffer = new Uint8Array(demo.model.names);
}

function readNameAt(demo, address) {
  if (!demo.namesBuffer || address < 0 || address >= demo.namesBuffer.length) { return ''; }
  let end = address;
  while (end < demo.namesBuffer.length && demo.namesBuffer[end] !== 0) { end++; }
  return demo.textDecoder.decode(demo.namesBuffer.subarray(address, end));
}

function findIdByName(demo, name, addressArray, count) {
  if (!name || !addressArray || count <= 0) { return -1; }
  for (let i = 0; i < count; i++) {
    const start = addressArray[i];
    const candidate = readNameAt(demo, start);
    if (candidate === name) { return i; }
  }
  return -1;
}

function getJointAddress(demo, name) {
  const jointId = findIdByName(demo, name, demo.model.name_jntadr, demo.model.njnt);
  if (jointId === -1) { return -1; }
  return demo.model.jnt_qposadr[jointId];
}

export {
  refreshNameBuffer,
  readNameAt,
  findIdByName,
  getJointAddress,
};
