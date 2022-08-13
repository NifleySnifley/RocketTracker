class TestEv1 extends EventTarget {
	dispatch(t, d) {
		this.dispatchEvent(new CustomEvent(t, { detail: d }));
	}
}

class TestEv2 extends EventTarget {
	constructor(tev1) {
		super();
		tev1.addEventListener('test1', e => this.dispatch('test2', { message: e.detail.message }));
	}

	dispatch(t, d) {
		this.dispatchEvent(new CustomEvent(t, { detail: d }));
	}
}

const tev1 = new TestEv1();
const tev2 = new TestEv2(tev1);

function sendEvs() {
	tev1.dispatch('test1', { message: 'got it!' });
	setTimeout(sendEvs, 1000);
}

sendEvs();
tev2.addEventListener('test2', e => console.log(e.detail.message));
